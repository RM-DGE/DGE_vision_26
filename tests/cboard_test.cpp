/**
 * @file cboard_test.cpp
 * @brief CBoard 双向串口测试：PC -> CBoard 控制帧，CBoard -> PC IMU/状态帧。
 *
 * 这个程序只做“串口通不通”的检查，不做 IMU 时间对齐、不做视觉：
 *   - TX：按 --tx_hz 周期发送 0xCC 控制帧（control/shoot/yaw/pitch + 0xEE）；
 *   - RX：用 CBoard 的 observer 统计 0xAA IMU 帧和 0xBB 状态帧；
 *   - 每秒打印一次 TX/RX 计数、RX 帧率、弹速/模式，看到两边都在动就说明
 *     串口、波特率、接线和下位机解析都正常；
 *   - --raw=1 时打印每一帧 TX 的 8 字节十六进制，便于对照下位机协议。
 *
 * 安全默认：
 *   control=0、shoot=0，只发“空指令”测 TX，不会驱动云台或发射。
 *   真要让云台响应，必须显式 --control=1 --yes=1（会动，注意安全）。
 *
 * 用法：
 *   ./build/cboard_test configs/standard3.yaml
 *   ./build/cboard_test configs/sentry.yaml --duration=30 --tx_hz=20
 *   ./build/cboard_test configs/standard3.yaml --tx_hz=0        # 只收不发
 *   ./build/cboard_test configs/standard3.yaml --raw=1          # 打印 TX 字节
 *   ./build/cboard_test configs/standard3.yaml --control=1 --yes=1 --yaw=0.2 --pitch=-0.1
 *
 * 退出码：
 *   0  双向正常（有 TX，且收到 RX 帧）
 *   1  参数/配置错误
 *   2  CBoard 构造失败
 *   3  没有收到任何 RX 帧（查设备、权限、波特率、TX/RX 交叉、GND、下位机供电）
 *   4  TX 一次都没发出去（检查 --tx_hz / 串口是否连接）
 */

#include "io/cboard.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace
{

const std::string keys =
  "{help h usage ? |                         | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml  | yaml 配置文件路径 }"
  "{duration d     | 10                      | 测试时长（秒），0 表示直到 Ctrl+C }"
  "{tx_hz          | 20                      | 控制帧发送频率；0 = 只收不发 }"
  "{control        | 0                       | 1 = control 置位（会驱动云台，需 --yes=1） }"
  "{shoot          | 0                       | 1 = shoot 置位（会尝试发射，需 --yes=1） }"
  "{yaw            | 0                       | 控制帧 yaw（rad） }"
  "{pitch          | 0                       | 控制帧 pitch（rad） }"
  "{yes            | 0                       | control/shoot=1 时的安全确认 }"
  "{raw            | 0                       | 1 = 打印每帧 TX 的十六进制 }";

auto mode_name(int mode) -> std::string
{
  if (mode >= 0 && mode < static_cast<int>(io::MODES.size())) return io::MODES[mode];
  return "unknown(" + std::to_string(mode) + ")";
}

// 仅用于显示：和 CBoard::send() 的组帧方式保持一致（8 字节，packed）。
auto control_frame_hex(const io::Command & command) -> std::string
{
  io::ControlDataFrame frame{};
  frame.header = 0xCC;
  frame.control = command.control ? 1 : 0;
  frame.shoot = command.shoot ? 1 : 0;
  frame.yaw = static_cast<int16_t>(command.yaw * 10000);
  frame.pitch = static_cast<int16_t>(command.pitch * 10000);
  frame.tail = 0xEE;

  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&frame);
  std::string text;
  for (std::size_t i = 0; i < sizeof(frame); ++i) {
    text += fmt::format("{:02X} ", static_cast<unsigned>(bytes[i]));
  }
  return text;
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);
  const auto duration_s = cli.get<double>("duration");
  const auto tx_hz = cli.get<double>("tx_hz");
  const bool control = cli.get<int>("control") != 0;
  const bool shoot = cli.get<int>("shoot") != 0;
  const auto yaw = cli.get<double>("yaw");
  const auto pitch = cli.get<double>("pitch");
  const bool confirmed = cli.get<int>("yes") != 0;
  const bool raw = cli.get<int>("raw") != 0;

  if (!std::isfinite(tx_hz) || tx_hz < 0.0) {
    std::cerr << "tx_hz must be >= 0\n";
    return 1;
  }
  if ((control || shoot) && !confirmed) {
    std::cerr << "control/shoot=1 会实际驱动下位机。确认后加 --yes=1 再运行。\n";
    return 1;
  }
  if (!std::filesystem::exists(config_path)) {
    std::cerr << "config not found: " << config_path << "\n";
    return 1;
  }

  std::string device;
  int baudrate = 115200;
  try {
    auto yaml = YAML::LoadFile(config_path);
    if (!yaml["serial_device"]) {
      std::cerr << "missing serial_device in " << config_path << "\n";
      return 1;
    }
    device = yaml["serial_device"].as<std::string>();
    if (yaml["serial_baudrate"]) baudrate = yaml["serial_baudrate"].as<int>();
  } catch (const std::exception & e) {
    std::cerr << "failed to read " << config_path << ": " << e.what() << "\n";
    return 1;
  }

  std::error_code ec;
  std::filesystem::create_directories("logs", ec);
  tools::logger()->set_level(spdlog::level::info);

  std::cout << fmt::format(
                 "[cboard_test] config={} device={} baud={} duration={}s tx_hz={} "
                 "control={} shoot={}\n",
                 config_path, device, baudrate, duration_s, tx_hz, control, shoot)
            << std::flush;

  tools::Exiter exiter;

  std::atomic<std::uint64_t> imu_frames{0};
  std::atomic<std::uint64_t> status_frames{0};
  std::atomic<std::uint64_t> generation_changes{0};

  std::mutex status_mutex;
  io::StatusSample latest_status{};

  std::unique_ptr<io::CBoard> cboard;
  try {
    cboard = std::make_unique<io::CBoard>(config_path);
  } catch (const std::exception & e) {
    std::cerr << "[cboard_test] open CBoard failed: " << e.what() << "\n";
    return 2;
  }

  cboard->set_imu_observer([&](const io::ImuSample &) {
    imu_frames.fetch_add(1, std::memory_order_relaxed);
  });
  cboard->set_status_observer([&](const io::StatusSample & sample) {
    status_frames.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(status_mutex);
    latest_status = sample;
  });

  io::Command command{};
  command.control = control;
  command.shoot = shoot;
  command.yaw = yaw;
  command.pitch = pitch;

  const auto start = std::chrono::steady_clock::now();
  const auto tx_period = tx_hz > 0.0
                           ? std::chrono::duration<double>(1.0 / tx_hz)
                           : std::chrono::duration<double>(0.0);

  auto next_tx = start;
  auto last_print = start;
  auto last_generation = cboard->connection_generation();

  std::uint64_t tx_frames = 0;

  while (!exiter.exit()) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - start).count();

    if (duration_s > 0.0 && elapsed >= duration_s) break;

    const auto generation = cboard->connection_generation();
    if (generation != last_generation) {
      last_generation = generation;
      generation_changes.fetch_add(1, std::memory_order_relaxed);
      tools::logger()->warn("[cboard_test] serial connection generation -> {}", generation);
    }

    if (tx_hz > 0.0 && now >= next_tx) {
      cboard->send(command);
      ++tx_frames;
      if (raw) std::cout << "[cboard_test] TX " << control_frame_hex(command) << std::endl;
      next_tx += std::chrono::duration_cast<std::chrono::steady_clock::duration>(tx_period);
      if (next_tx < now) {
        next_tx =
          now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(tx_period);
      }
    }

    if (now - last_print >= 1s) {
      const auto imu_count = imu_frames.load(std::memory_order_relaxed);
      const auto status_count = status_frames.load(std::memory_order_relaxed);

      io::StatusSample status{};
      {
        std::lock_guard<std::mutex> lock(status_mutex);
        status = latest_status;
      }

      tools::logger()->info(
        "[cboard_test] t={:.1f}s tx={} rx_imu={} ({:.0f}Hz) rx_status={} | "
        "speed={:.2f} mode={} shoot_mode={} ft={:+.2f}",
        elapsed, tx_frames, imu_count,
        elapsed > 0.0 ? static_cast<double>(imu_count) / elapsed : 0.0, status_count,
        status.bullet_speed, mode_name(status.mode), status.shoot_mode, status.ft_angle);

      last_print = now;
    }

    std::this_thread::sleep_for(5ms);
  }

  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  const auto imu_count = imu_frames.load(std::memory_order_relaxed);
  const auto status_count = status_frames.load(std::memory_order_relaxed);

  std::cout << fmt::format(
                 "\n[cboard_test] ---- summary ----\n"
                 "  config        : {}\n"
                 "  device/baud   : {} @ {}\n"
                 "  elapsed       : {:.2f} s\n"
                 "  tx frames     : {} ({:.1f} Hz)\n"
                 "  rx imu        : {} ({:.1f} Hz)\n"
                 "  rx status     : {} ({:.1f} Hz)\n"
                 "  reconnects    : {}\n",
                 config_path, device, baudrate, elapsed, tx_frames,
                 elapsed > 0.0 ? static_cast<double>(tx_frames) / elapsed : 0.0, imu_count,
                 elapsed > 0.0 ? static_cast<double>(imu_count) / elapsed : 0.0, status_count,
                 elapsed > 0.0 ? static_cast<double>(status_count) / elapsed : 0.0,
                 generation_changes.load(std::memory_order_relaxed))
            << std::flush;

  if (imu_count == 0) {
    std::cerr << "\n[cboard_test] 没有收到任何 RX 帧。检查：\n"
                 "  1) 设备是否存在：ls -l " << device << "\n"
                 "  2) 当前用户是否有串口权限：sudo usermod -aG dialout $USER（重新登录）\n"
                 "  3) 波特率是否与下位机一致（常见 115200）\n"
                 "  4) TX/RX 是否交叉、GND 是否共地、下位机是否上电\n"
                 "  5) 串口是否被其他进程占用（同一设备只能一个 owner）\n";
    return 3;
  }

  if (tx_hz > 0.0 && tx_frames == 0) {
    std::cerr << "\n[cboard_test] 没有发出任何 TX 帧（检查 --tx_hz 和串口连接状态）。\n";
    return 4;
  }

  std::cout << "\n[cboard_test] 双向测试通过：TX 在发，RX 在收。\n";
  return 0;
}

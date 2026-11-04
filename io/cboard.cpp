/* C板通信模块
 *
 * IMU数据帧10字节：
 * 帧头：0xAA
 * 数据：w, x, y, z（每个为int16_t，接收实际值是乘以10000），帧尾0xDD
 *
 * 状态数据帧9字节：
 * 帧头：0xBB
 * 数据：bullet_speed、mode、ShootMode、ft_angle（各为int16_t）
 *
 * 发送控制指令数据帧8字节：
 * - 帧头：0xCC
 * - 数据：control（uint8_t）、shoot（uint8_t）、yaw（int16_t）、pitch（int16_t），帧尾0xEE
 */
/* C板通信模块 */
#include "cboard.hpp"

#include <utility>

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{

//初始化C板通信
CBoard::CBoard(const std::string & config_path)
: serial_(read_yaml(config_path), read_baudrate(config_path),
          std::bind(&CBoard::callback, this, std::placeholders::_1))
{
  tools::logger()->info("[Cboard] Opened; alignment waits for real IMU samples.");
}

// 根据给定的时间戳，通过插值计算出该时刻对应的 IMU 四元
tools::time_sync::ImuQuery CBoard::imu_query(
  std::chrono::steady_clock::time_point timestamp, double max_gap_ms, double max_wait_ms)
{
  if (imu_generation_.load() != serial_.connection_generation()) return {};
  return imu_buffer_.wait_query(timestamp, max_gap_ms, max_wait_ms);
}

// 发送控制指令到C板
void CBoard::send(Command command) const
{
  ControlDataFrame frame;
  frame.header = 0xCC;
  
  frame.control = (command.control) ? 1 : 0;
  frame.shoot = (command.shoot) ? 1 : 0;
  frame.yaw = (int16_t)(command.yaw * 10000);
  frame.pitch = (int16_t)(command.pitch * 10000);
  frame.tail = 0xEE;
  // frame.yaw_vel = (int16_t)(command.yaw_vel * 1000);
  // frame.pitch_vel = (int16_t)(command.pitch_vel * 1000);
  // frame.horizon_distance = (int16_t)(command.horizon_distance * 10000);

  std::vector<uint8_t> data(sizeof(frame));
  std::memcpy(data.data(), &frame, sizeof(frame));

  try 
  {
    serial_.write(data);
  } catch (const std::exception & e) {
    tools::logger()->warn("Serial write failed: {}", e.what());
  }
}

// 解析并存储IMU和状态数据
void CBoard::callback(const std::vector<uint8_t> & frame)
{
  auto timestamp = std::chrono::steady_clock::now();
  
  if (frame.empty()) return;
  const auto generation = serial_.connection_generation();
  if (generation != imu_generation_.load()) {
    imu_buffer_.clear();
    imu_generation_ = generation;
  }
  uint8_t header = frame[0];

  if (header == 0xAA) {
    if (frame.size() != sizeof(IMUDataFrame)) {
      tools::logger()->warn("IMU frame size mismatch: expected {}, got {}", sizeof(IMUDataFrame), frame.size());
      return;
    }
    
    const auto* data = reinterpret_cast<const IMUDataFrame*>(frame.data());
    
    if (data->tail != 0xDD) { 
      tools::logger()->warn("Invalid IMU frame tail: expected 0x55, got 0x{:02X}", data->tail);
      return; // 帧尾不对，说明发生了数据错位，直接丢弃该帧
    }

    double w = data->w / 10000.0;
    double x = data->x / 10000.0;
    double y = data->y / 10000.0;
    double z = data->z / 10000.0;
    // double w = data->w / 10000.0;

    double norm_sq = x * x + y * y + z * z + w * w;
    if (std::abs(norm_sq - 1.0) > 0.1) {
      tools::logger()->warn("Invalid quaternion norm: {:.4f}", norm_sq);
      return;
    }

    if (!imu_buffer_.push({timestamp, Eigen::Quaterniond{w, x, y, z}})) {
      tools::logger()->warn("Rejected invalid or non-monotonic IMU receive sample");
      return;
    }

    // ROS 桥接层观察者：只做 mailbox 更新，不在这里 publish。
    {
        std::lock_guard<std::mutex> lock(observer_mutex_);
        if (imu_observer_) {
          imu_observer_(ImuSample{Eigen::Quaterniond{w, x, y, z}, timestamp});
        }
    }
  } 
  else if (header == 0xBB) {
    if (frame.size() != sizeof(StatusDataFrame)) {
      tools::logger()->warn("Status frame size mismatch");
      return;
    }

    const auto* data = reinterpret_cast<const StatusDataFrame*>(frame.data());

    bullet_speed = data->bullet_speed / 100.0;
    int m = data->mode;
    if (m >= 0 && m < static_cast<int>(MODES.size())) mode = Mode(m);
    shoot_mode = data->ShootMode;

    ft_angle = data->ft_angle / 10000.0;

    // ROS 桥接层观察者：状态低频，直接拷贝即可。
    {
        std::lock_guard<std::mutex> lock(observer_mutex_);
        if (status_observer_) {
          status_observer_(StatusSample{
            bullet_speed, static_cast<int>(mode), shoot_mode, ft_angle, timestamp});
        }
    }

    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    if (tools::delta_time(timestamp, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Speed: {:.2f}, Mode: {}, Shoot: {}", //, FT: {:.2f}",
        bullet_speed);//, ft_angle);, SHOOT_MODES[shoot_mode]
      last_log_time = timestamp;
    }
  } else {
    tools::logger()->debug("Unknown frame header: 0x{:02X}", header);
  }
}

void CBoard::set_imu_observer(ImuObserver observer)
{
  std::lock_guard<std::mutex> lock(observer_mutex_);
  imu_observer_ = std::move(observer);
}

void CBoard::set_status_observer(StatusObserver observer)
{
  std::lock_guard<std::mutex> lock(observer_mutex_);
  status_observer_ = std::move(observer);
}

std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  return yaml["serial_device"].as<std::string>();
}

int CBoard::read_baudrate(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  if (!yaml["serial_baudrate"]) {
    return 115200;
  }
  return yaml["serial_baudrate"].as<int>();
}

}  // namespace io

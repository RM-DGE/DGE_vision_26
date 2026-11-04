#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "tools/time_sync_recorder.hpp"
#ifdef TIME_SYNC_IO_TESTS
#include "io/Daedalus/shared.hpp"
#include "io/cboard.hpp"
#endif
// v0.51 regression suite; grouped by responsibility.

namespace recorder_checks
{

using namespace tools::time_sync;
namespace fs = std::filesystem;
using namespace std::chrono_literals;
void check(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
std::vector<std::string> lines(const fs::path & path)
{
  std::ifstream stream(path);
  std::vector<std::string> result;
  std::string line;
  while (std::getline(stream, line)) result.push_back(line);
  return result;
}
YAML::Node geometry()
{
  return YAML::Load(R"(
camera_matrix: [1000,0,320,0,1000,240,0,0,1]
distort_coeffs: [0,0,0,0,0]
R_camera2gimbal: [1,0,0,0,1,0,0,0,1]
t_camera2gimbal: [0,0,0]
R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]
time_sync_board: {columns: 9, rows: 6, square_size_m: 0.03}
)");
}
RecorderOptions options(const fs::path & directory)
{
  RecorderOptions result;
  result.output = directory;
  result.calibration = geometry();
  result.t0 = std::chrono::steady_clock::now();
  result.image_capacity = 64;
  return result;
}
int run(int argc, char ** argv)
{
  try {
    // Keep generated evidence; the test never recursively removes temporary data.
    const fs::path root = argc > 1 ? argv[1] : fs::temp_directory_path();
    const auto id = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto base = root / ("time-sync-recorder-test-" + id);
    fs::create_directories(base);
    {
      auto cfg = options(base / "drain");
      TimeSyncRecorder recorder(cfg);
      cv::Mat pixels(8, 8, CV_8UC1, cv::Scalar(17));
      ImageMetadata metadata;
      metadata.camera_frame_id = 42;
      metadata.device_timestamp_ticks = 123456;
      metadata.exposure_us = 3000;
      check(recorder.record_image(pixels, cfg.t0 + 3ms, metadata), "accept first image");
      pixels.setTo(99);
      check(recorder.record_image(pixels, cfg.t0 + 43ms), "accept second image");
      pixels.setTo(0);
      for (int i = 0; i < 100; ++i)
        check(
          recorder.record_imu(Eigen::Quaterniond(1.03125, 0, 0, 0), cfg.t0 + i * 10ms),
          "accept raw IMU");
      const auto stats = recorder.finish();
      check(
        stats.complete && stats.images_written == 2 && stats.imu_written == 100,
        "finish drains accepted queues");
      check(recorder.finish().complete, "finish is idempotent");
      check(
        !recorder.record_imu(Eigen::Quaterniond::Identity(), cfg.t0 + 2s),
        "closed producer rejected");
      auto csv = lines(cfg.output / "images.csv");
      check(
        csv.size() == 3 && csv[1] == "0,3000000,images/0.png,42,123456,3000",
        "raw time and explicit device metadata mapped");
      check(csv[2] == "1,43000000,images/1.png,,,", "unavailable metadata stays empty");
      check(
        cv::imread((cfg.output / "images/0.png").string(), 0).at<unsigned char>(0, 0) == 17,
        "first image owns copy");
      check(
        cv::imread((cfg.output / "images/1.png").string(), 0).at<unsigned char>(0, 0) == 99,
        "second image owns copy");
      check(
        lines(cfg.output / "imu.csv")[1] == "0,0,1.03125,0,0,0,1",
        "raw wxyz preserved without normalization");
      const auto session = YAML::LoadFile((cfg.output / "session.yaml").string());
      check(
        session["complete"].as<bool>() &&
          session["calibration_fingerprint"].as<std::string>().size() > 5,
        "complete metadata and fingerprint");
      bool rejected = false;
      try {
        TimeSyncRecorder duplicate(cfg);
      } catch (const std::exception &) {
        rejected = true;
      }
      check(rejected, "existing session must not be overwritten");
    }
    {
      auto cfg = options(base / "downsample");
      cfg.image_save_fps = 10;
      TimeSyncRecorder recorder(cfg);
      cv::Mat pixels(8, 8, CV_8UC1, cv::Scalar(3));
      for (int i = 0; i < 10; ++i) {
        recorder.record_image(pixels, cfg.t0 + i * 10ms);
        check(
          recorder.record_imu(Eigen::Quaterniond::Identity(), cfg.t0 + i * 10ms),
          "IMU independent of image decimation");
      }
      const auto stats = recorder.finish();
      check(
        stats.complete && stats.images_received == 10 && stats.images_written == 1 &&
          stats.images_skipped == 9 && stats.imu_written == 10,
        "image decimation must preserve complete IMU sequence");
    }
    {
      auto cfg = options(base / "overflow");
      cfg.imu_capacity = 1;
      cfg.event_capacity = 1;
      TimeSyncRecorder recorder(cfg);
      for (int i = 0; i < 10000; ++i)
        recorder.record_imu(Eigen::Quaterniond::Identity(), cfg.t0 + i * 10ms);
      const auto stats = recorder.finish();
      check(!stats.complete && stats.imu_dropped > 0, "bounded logger must expose overload");
      check(
        stats.imu_received == stats.imu_accepted + stats.imu_dropped &&
          stats.imu_accepted == stats.imu_written + stats.imu_failed,
        "IMU overload counters conserved");
      const auto events = lines(cfg.output / "events.csv");
      bool reported = false;
      for (const auto & event : events)
        if (event.find("queue_overflow") != std::string::npos) reported = true;
      check(reported, "overflow must have a persisted event");
    }
    {
      auto cfg = options(base / "invalid");
      TimeSyncRecorder recorder(cfg);
      check(
        recorder.record_imu(Eigen::Quaterniond(0, 0, 0, 0), cfg.t0 + 1ms),
        "invalid quaternion logged");
      check(
        recorder.record_imu(Eigen::Quaterniond::Identity(), cfg.t0), "backwards timestamp logged");
      check(!recorder.record_image(cv::Mat(), cfg.t0), "empty frame rejected");
      const auto stats = recorder.finish();
      check(
        !stats.complete && stats.imu_invalid == 2 && stats.imu_written == 2,
        "invalid data is explicit incomplete");
      check(lines(cfg.output / "imu.csv")[1] == "0,1000000,0,0,0,0,0", "invalid flag serialized");
      check(lines(cfg.output / "events.csv").size() >= 4, "invalid data has events");
    }
    {
      auto cfg = options(base / "nonunit");
      TimeSyncRecorder recorder(cfg);
      check(
        recorder.record_imu(Eigen::Quaterniond(2, 0, 0, 0), cfg.t0),
        "bad norm retained for diagnostics");
      const auto stats = recorder.finish();
      check(
        !stats.complete && stats.imu_invalid == 1,
        "recorder must use the runtime/solver norm tolerance");
      check(lines(cfg.output / "imu.csv")[1] == "0,0,2,0,0,0,0", "bad norm cannot be marked valid");
    }
    {
      auto cfg = options(base / "io-failure");
      TimeSyncRecorder recorder(cfg);
      // A directory at the exact PNG destination is a deterministic real I/O failure.
      fs::create_directory(cfg.output / "images/0.png");
      check(
        recorder.record_image(cv::Mat(8, 8, CV_8UC1, cv::Scalar(7)), cfg.t0),
        "failure sample accepted");
      const auto stats = recorder.finish();
      check(
        !stats.complete && stats.images_failed == 1 && stats.images_written == 0,
        "write failure incomplete and conserved");
      check(lines(cfg.output / "images.csv").size() == 1, "failed PNG has no successful CSV row");
      check(
        !YAML::LoadFile((cfg.output / "session.yaml").string())["complete"].as<bool>(),
        "incomplete persisted");
    }
    {
      auto cfg = options(base / "abandoned");
      {
        TimeSyncRecorder recorder(cfg);
        recorder.record_imu(Eigen::Quaterniond::Identity(), cfg.t0);
      }
      check(
        !YAML::LoadFile((cfg.output / "session.yaml").string())["complete"].as<bool>(),
        "implicit destructor cannot certify clean stop");
      check(lines(cfg.output / "imu.csv").size() == 2, "destructor still drains");
    }
    {
      auto bad = geometry();
      bad["time_sync_board"]["square_size_m"] = -1;
      bool rejected = false;
      try {
        validate_calibration_geometry(bad);
      } catch (const std::exception &) {
        rejected = true;
      }
      check(rejected, "bad board rejected");
      bad = geometry();
      bad["R_camera2gimbal"][0] = 2;
      rejected = false;
      try {
        validate_calibration_geometry(bad);
      } catch (const std::exception &) {
        rejected = true;
      }
      check(rejected, "nonrotation extrinsic rejected");
      for (const auto & key : {"R_camera2gimbal", "R_gimbal2imubody"}) {
        bad = geometry();
        bad[key][0] = 1.0001;
        rejected = false;
        try {
          validate_calibration_geometry(bad);
        } catch (const std::exception &) {
          rejected = true;
        }
        check(rejected, "recorder and solver must share rotation tolerance");
      }
      bad = geometry();
      bad["time_sync_board"]["columns"] = 101;
      rejected = false;
      try {
        validate_calibration_geometry(bad);
      } catch (const std::exception &) {
        rejected = true;
      }
      check(rejected, "board size must be accepted by both recorder and solver");
      bad = geometry();
      bad["camera_matrix"][1] = 0.1;
      rejected = false;
      try {
        validate_calibration_geometry(bad);
      } catch (const std::exception &) {
        rejected = true;
      }
      check(rejected, "OpenCV camera matrix must have zero skew");
    }
    std::cout << "time_sync_recorder_test passed; evidence: " << base << '\n';
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

}  // namespace recorder_checks
#ifdef TIME_SYNC_IO_TESTS

namespace sim_checks
{

using namespace std::chrono_literals;
using tools::time_sync::ImuQueryStatus;
void check(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

struct TestBoard : io::CBoardBase
{
  tools::time_sync::ImuTimeBuffer buffer;
  tools::time_sync::ImuQuery imu_query(tools::time_sync::TimePoint t, double gap, double) override
  {
    return buffer.query(t, gap);
  }
  void send(io::Command) const override {}
};

int run()
{
  try {
    const auto t = tools::time_sync::TimePoint{} + 1s;
    TestBoard board;
    bool rejected = false;
    try {
      board.imu_at(t);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    check(rejected, "compatibility API must fail explicitly without an attitude");
    board.buffer.push({t, Eigen::Quaterniond::Identity()});
    check(board.imu_at(t).isApprox(Eigen::Quaterniond::Identity()), "compatibility exact hit");

    // Local shared state needs no simulator, camera SDK, serial port, or IPC files.
    io::DaedalusShared shared;
    int observed = 0;
    shared.imu_observer = [&](const Eigen::Quaterniond &, tools::time_sync::TimePoint stamp) {
      ++observed;
      check(stamp == t, "image and matched pose share one timestamp");
    };
    check(!shared.publish_frame_imu(10, t), "missing pose must not become identity IMU");
    shared.stage_gimbal_pose(Eigen::Quaterniond::Identity(), 9);
    check(!shared.publish_frame_imu(10, t), "old pose cannot receive current frame timestamp");
    shared.stage_gimbal_pose(Eigen::Quaterniond::Identity(), 11);
    check(!shared.publish_frame_imu(10, t), "future pose cannot label an earlier image");
    check(shared.publish_frame_imu(11, t), "matching sequence releases a real sample");
    check(
      observed == 1 && shared.imu_buffer.query(t, 20).valid(),
      "matched pose observed and buffered");
    check(!shared.publish_frame_imu(12, t + 10ms), "sample cannot be reused by another frame");
    check(
      shared.imu_buffer.query(t + 10ms, 20).status == ImuQueryStatus::missing_right,
      "missing next pose remains an explicit query failure");
    shared.stage_gimbal_pose(Eigen::Quaterniond(0, 0, 0, 0), 12);
    check(
      !shared.publish_frame_imu(12, t + 10ms) && observed == 1, "invalid pose is never published");
    std::cout << "time_sync_io_test passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

}  // namespace sim_checks
#endif
#ifdef TIME_SYNC_IO_TESTS

namespace serial_checks
{

using namespace std::chrono_literals;
namespace fs = std::filesystem;
void check(bool condition, const char * why)
{
  if (!condition) throw std::runtime_error(why);
}
struct Pty
{
  int fd = posix_openpt(O_RDWR | O_NOCTTY);
  std::string slave;
  Pty()
  {
    check(fd >= 0 && grantpt(fd) == 0 && unlockpt(fd) == 0, "create pseudo-terminal");
    slave = ptsname(fd);
  }
  ~Pty()
  {
    if (fd >= 0) close(fd);
  }
  void shutdown()
  {
    close(fd);
    fd = -1;
  }
  void imu(int16_t w = 10000)
  {
    const io::IMUDataFrame frame{0xAA, w, 0, 0, 0, 0xDD};
    check(write(fd, &frame, sizeof(frame)) == sizeof(frame), "write simulated serial frame");
  }
};

int run()
{
  try {
    const auto root = fs::temp_directory_path() /
                      ("time-sync-serial-" +
                       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root);
    Pty first, second;
    fs::create_symlink(first.slave, root / "port");
    {
      std::ofstream config(root / "config.yaml");
      config << "serial_device: " << (root / "port").string() << "\nserial_baudrate: 115200\n";
    }
    io::CBoard board((root / "config.yaml").string());
    check(board.connection_generation() == 1, "first connection epoch");
    check(!board.imu_query(std::chrono::steady_clock::now(), 20, 0).valid(), "no seeded identity");
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<io::ImuSample> received;
    board.set_imu_observer([&](const io::ImuSample & sample) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(sample);
      changed.notify_all();
    });
    struct Detach
    {
      io::CBoard & board;
      ~Detach() { board.set_imu_observer({}); }
    } detach{board};
    const auto receive = [&](std::size_t count) {
      std::unique_lock<std::mutex> lock(mutex);
      check(
        changed.wait_for(lock, 2s, [&] { return received.size() >= count; }),
        "serial observer timeout");
      return received[count - 1];
    };
    first.imu();
    const auto old = receive(1);
    check(board.imu_query(old.timestamp, 20, 0).valid(), "exact received frame query");
    board.send(io::Command{false, false, 0, 0});
    pollfd command_ready{first.fd, POLLIN, 0};
    check(poll(&command_ready, 1, 1000) == 1, "control packet is transmitted");
    io::ControlDataFrame disabled{};
    check(
      read(first.fd, &disabled, sizeof(disabled)) == sizeof(disabled),
      "control packet length unchanged");
    check(
      disabled.header == 0xCC && disabled.tail == 0xEE && !disabled.control && !disabled.shoot,
      "disabled control wire format unchanged");
    first.imu(20000);  // Must be rejected before observer/buffer.
    std::this_thread::sleep_for(10ms);
    const io::IMUDataFrame split{0xAA, 10000, 0, 0, 0, 0xDD};
    const auto * raw = reinterpret_cast<const unsigned char *>(&split);
    check(write(first.fd, raw, 4) == 4, "write partial frame");
    std::this_thread::sleep_for(10ms);
    {
      std::lock_guard<std::mutex> lock(mutex);
      check(received.size() == 1, "partial frame is not a sample");
    }
    check(write(first.fd, raw + 4, sizeof(split) - 4) == sizeof(split) - 4, "finish partial frame");
    const auto next = receive(2);
    check(next.q.isApprox(Eigen::Quaterniond::Identity()), "bad wire quaternion excluded");

    // Replace only this fixture's port symlink, then force a real disconnect.
    fs::create_symlink(second.slave, root / "next-port");
    fs::rename(root / "next-port", root / "port");
    first.shutdown();
    const auto deadline = std::chrono::steady_clock::now() + 6s;
    while (board.connection_generation() == 1 && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(10ms);
    check(board.connection_generation() == 2, "reconnected port has a new epoch");
    check(
      !board.imu_query(old.timestamp, 20, 0).valid(),
      "old epoch cannot be queried after reconnect");
    second.imu();
    const auto fresh = receive(3);
    check(board.imu_query(fresh.timestamp, 20, 0).valid(), "new epoch sample is usable");
    check(
      !board.imu_query(old.timestamp, 20, 0).valid(), "buffer cannot interpolate across reconnect");
    board.set_imu_observer({});
    second.imu();
    std::this_thread::sleep_for(10ms);
    {
      std::lock_guard<std::mutex> lock(mutex);
      check(received.size() == 3, "unregistered observer remains silent");
    }
    std::cout << "time_sync_serial_test passed; retained fixtures: " << root << '\n';
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

}  // namespace serial_checks
#endif

int main()
{
  if (recorder_checks::run(0, nullptr) != 0) return 1;
#ifdef TIME_SYNC_IO_TESTS
  if (sim_checks::run() != 0) return 1;
#endif
#ifdef TIME_SYNC_IO_TESTS
  if (serial_checks::run() != 0) return 1;
#endif
  return 0;
}

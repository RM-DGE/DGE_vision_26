#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "io/Daedalus/daedalus_cboard.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/time_sync_recorder.hpp"

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
using Clock = std::chrono::steady_clock;
using tools::time_sync::TimeSyncRecorder;
std::map<std::string, std::string> arguments(int argc, char ** argv)
{
  std::map<std::string, std::string> result;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--help") {
      result[key] = "";
      continue;
    }
    if (
      key != "--config" && key != "--output" && key != "--duration" && key != "--image-fps" &&
      key != "--cols" && key != "--rows" && key != "--square-m")
      throw std::invalid_argument("unknown option " + key);
    if (i + 1 >= argc || result.count(key))
      throw std::invalid_argument("missing or duplicate value for " + key);
    result[key] = argv[++i];
  }
  return result;
}
double number(
  const std::map<std::string, std::string> & args, const std::string & name, double fallback)
{
  const auto found = args.find(name);
  if (found == args.end()) return fallback;
  std::size_t used = 0;
  const auto result = std::stod(found->second, &used);
  if (used != found->second.size() || !std::isfinite(result))
    throw std::invalid_argument("invalid value for " + name);
  return result;
}
int board_count(
  const std::map<std::string, std::string> & args, const std::string & name, int fallback)
{
  const auto value = number(args, name, fallback);
  if (value != std::floor(value) || value < 2 || value > 100)
    throw std::invalid_argument("board count must be an integer in [2,100]: " + name);
  return static_cast<int>(value);
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto args = arguments(argc, argv);
    if (args.count("--help")) {
      std::cout
        << "capture_time_sync --config CONFIG --output FRESH_SESSION_DIR "
           "[--duration 30] [--image-fps 30] --cols COLS --rows ROWS --square-m METERS\n"
           "Board options may instead come from config time_sync_board. The operator moves the "
           "gimbal; "
           "this program sends no control commands. Ctrl-C stops and drains accepted records.\n";
      return 0;
    }
    if (!args.count("--config") || !args.count("--output"))
      throw std::invalid_argument("--config and --output are required (see --help)");
    const auto config_path = args.at("--config");
    const auto yaml = YAML::LoadFile(config_path);
    auto snapshot = YAML::Clone(yaml);
    const auto board = yaml["time_sync_board"];
    // Explicit geometry must exist before constructing a device or output session.
    snapshot["time_sync_board"]["columns"] =
      board_count(args, "--cols", board && board["columns"] ? board["columns"].as<int>() : 0);
    snapshot["time_sync_board"]["rows"] =
      board_count(args, "--rows", board && board["rows"] ? board["rows"].as<int>() : 0);
    snapshot["time_sync_board"]["square_size_m"] = number(
      args, "--square-m",
      board && board["square_size_m"] ? board["square_size_m"].as<double>() : 0);
    snapshot["length_unit"] = "m";
    tools::time_sync::validate_calibration_geometry(snapshot);

    tools::time_sync::RecorderOptions options;
    options.output = std::filesystem::absolute(args.at("--output"));
    options.calibration = snapshot;
    options.duration_s = number(args, "--duration", 30);
    options.image_save_fps = number(args, "--image-fps", 30);
    options.camera_name = yaml["camera_name"].as<std::string>();
    options.exposure_us = yaml["exposure_ms"].as<double>() * 1000;
    if (options.camera_name != "hikrobot" && options.camera_name != "daedalus")
      throw std::invalid_argument("collector supports only hikrobot and daedalus camera backends");
    if (
      options.duration_s <= 0 || options.duration_s > 86400 || options.image_save_fps <= 0 ||
      options.image_save_fps > 1000 || !std::isfinite(options.exposure_us) ||
      options.exposure_us <= 0)
      throw std::invalid_argument("invalid duration, image-fps, or exposure_ms");
    options.timestamp_semantics =
      options.camera_name == "daedalus" ? "daedalus_frame_v1" : "host_receive_v1";
#ifdef TIME_SYNC_SOURCE_HEAD
    options.source_head = TIME_SYNC_SOURCE_HEAD;
#endif
    // This one t0 precedes opening either sensor and is used for every stream.
    options.t0 = Clock::now();
    TimeSyncRecorder recorder(options);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    bool clean = true;
    try {
      std::unique_ptr<io::CBoardBase> board_io;
      if (options.camera_name == "daedalus")
        board_io = std::make_unique<io::DaedalusCBoard>(config_path);
      else
        board_io = std::make_unique<io::CBoard>(config_path);
      board_io->set_imu_observer([&recorder](const io::ImuSample & sample) {
        // This callback is independent of image decimation. No filesystem I/O.
        recorder.record_imu(sample.q, sample.timestamp);
      });
      auto imu_generation = board_io->connection_generation();
      const auto check_imu_connection = [&] {
        const auto current = board_io->connection_generation();
        if (imu_generation && current != imu_generation) {
          recorder.record_event(Clock::now(), "imu_reconnect", current, "start a new session");
          throw std::runtime_error("IMU serial port reconnected; start a new session");
        }
        imu_generation = current;
      };
      try {
        io::Camera camera(config_path);
        const auto started = Clock::now();
        const auto deadline = started + std::chrono::duration_cast<Clock::duration>(
                                          std::chrono::duration<double>(options.duration_s));
        auto last_frame_time = started;
        std::optional<std::uint64_t> previous_frame, previous_generation;
        std::optional<Clock::time_point> previous_time;
        std::uint64_t source_drops = 0, frame_count = 0;
        recorder.record_event(started, "capture_started", 0, options.timestamp_semantics);
        std::cout << "Recording " << options.output << "; move the gimbal manually.\n";
        while (!stop_requested && Clock::now() < deadline && !recorder.failed()) {
          check_imu_connection();
          cv::Mat image;
          Clock::time_point timestamp;
          if (!camera.read_for(image, timestamp, std::chrono::milliseconds(100))) {
            if (Clock::now() - last_frame_time >= std::chrono::seconds(5))
              throw std::runtime_error("camera has delivered no image for five seconds");
            continue;
          }
          last_frame_time = Clock::now();
          ++frame_count;
          const auto metadata = camera.frame_metadata();
          if (previous_time && timestamp <= *previous_time)
            throw std::runtime_error("camera host timestamp did not increase");
          previous_time = timestamp;
          if (previous_generation && metadata.capture_generation != *previous_generation)
            throw std::runtime_error("camera reconnected; start a new session");
          previous_generation = metadata.capture_generation;
          if (metadata.queue_dropped > source_drops) {
            recorder.record_source_drops(metadata.queue_dropped - source_drops);
            source_drops = metadata.queue_dropped;
          }
          if (previous_frame && metadata.frame_id) {
            if (*metadata.frame_id <= *previous_frame)
              throw std::runtime_error("camera source frame counter reset; start a new session");
            if (*metadata.frame_id > *previous_frame + 1)
              recorder.record_event(
                timestamp, "camera_frame_gap", frame_count - 1,
                "unobserved source frames: " +
                  std::to_string(*metadata.frame_id - *previous_frame - 1));
          }
          previous_frame = metadata.frame_id;
          tools::time_sync::ImageMetadata logged;
          logged.camera_frame_id = metadata.frame_id;
          logged.device_timestamp_ticks = metadata.device_timestamp_ticks;
          logged.exposure_us = metadata.exposure_us;
          recorder.record_image(
            image, timestamp, logged);  // original timestamp, no delay/half exposure applied
        }
        recorder.record_event(
          Clock::now(), "capture_stopped", frame_count,
          stop_requested ? "signal" : "duration_or_fault");
        check_imu_connection();
        if (!frame_count) recorder.mark_incomplete("no camera frames were received");
      } catch (...) {
        board_io->set_imu_observer({});  // synchronize with in-flight callbacks before draining
        throw;
      }
      board_io->set_imu_observer({});
    } catch (const std::exception & e) {
      clean = false;
      recorder.mark_incomplete(e.what());
      std::cerr << e.what() << '\n';
    }
    if (!recorder.stats().imu_received) recorder.mark_incomplete("no IMU records were received");
    const auto stats = recorder.finish(clean);
    std::cout << "complete=" << (stats.complete ? "true" : "false")
              << " images=" << stats.images_written << " imu=" << stats.imu_written
              << " image_drops=" << stats.images_dropped << " imu_drops=" << stats.imu_dropped
              << '\n';
    return stats.complete ? 0 : 2;
  } catch (const std::exception & e) {
    std::cerr << "capture_time_sync: " << e.what() << '\n';
    return 1;
  }
}

#include "tools/time_sync_recorder.hpp"

#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <thread>
#include <vector>

#include "tools/time_sync_session.hpp"

namespace tools::time_sync
{
namespace
{
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
std::vector<double> values(const YAML::Node & node, const char * key, std::size_t size)
{
  const auto field = node[key];
  if (!field || !field.IsSequence() || field.size() != size)
    throw std::invalid_argument(std::string("invalid calibration field: ") + key);
  auto data = field.as<std::vector<double>>();
  for (auto value : data)
    if (!std::isfinite(value))
      throw std::invalid_argument(std::string("nonfinite calibration field: ") + key);
  return data;
}
std::string csv_string(const std::string & value)
{
  std::string result = "\"";
  for (auto c : value) {
    if (c == '"') result += '"';
    result += (c == '\n' || c == '\r') ? ' ' : c;
  }
  return result + '"';
}
void checked_write(const std::filesystem::path & path, const std::string & text)
{
  std::ofstream stream(path);
  stream << text;
  stream.flush();
  if (!stream) throw std::runtime_error("unable to write " + path.string());
}
}  // namespace

void validate_calibration_geometry(const YAML::Node & node)
{
  if (!node.IsMap()) throw std::invalid_argument("calibration must be a map");
  const auto k = values(node, "camera_matrix", 9);
  if (
    k[0] <= 0 || k[4] <= 0 || std::abs(k[1]) >= 1e-12 || std::abs(k[3]) >= 1e-12 ||
    std::abs(k[6]) >= 1e-12 || std::abs(k[7]) >= 1e-12 || std::abs(k[8] - 1) >= 1e-12)
    throw std::invalid_argument("invalid camera matrix");
  const auto distortion = node["distort_coeffs"];
  if (!distortion || !distortion.IsSequence()) throw std::invalid_argument("missing distortion");
  const auto n = distortion.size();
  if (n != 4 && n != 5 && n != 8 && n != 12 && n != 14)
    throw std::invalid_argument("unsupported distortion coefficient count");
  values(node, "distort_coeffs", n);
  for (const auto * name : {"R_camera2gimbal", "R_gimbal2imubody"}) {
    const auto data = values(node, name, 9);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(data.data());
    if (
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() >= 1e-5 ||
      std::abs(rotation.determinant() - 1) >= 1e-5)
      throw std::invalid_argument(std::string("invalid rotation: ") + name);
  }
  values(node, "t_camera2gimbal", 3);
  const auto board = node["time_sync_board"];
  if (!board || !board.IsMap() || !board["columns"] || !board["rows"] || !board["square_size_m"])
    throw std::invalid_argument("time_sync_board columns/rows/square_size_m are required");
  const double square = board["square_size_m"].as<double>();
  if (
    board["columns"].as<int>() < 2 || board["rows"].as<int>() < 2 ||
    board["columns"].as<int>() > 100 || board["rows"].as<int>() > 100 || !std::isfinite(square) ||
    square <= 0)
    throw std::invalid_argument("invalid chessboard geometry");
}

struct TimeSyncRecorder::Impl
{
  struct Image
  {
    std::uint64_t id;
    std::int64_t time;
    cv::Mat pixels;
    ImageMetadata metadata;
  };
  struct Imu
  {
    std::uint64_t id;
    std::int64_t time;
    Eigen::Quaterniond q;
    bool valid;
  };
  struct Event
  {
    std::int64_t time;
    std::string type;
    std::uint64_t id;
    std::string reason;
  };
  RecorderOptions options;
  mutable std::mutex mutex;
  std::mutex finish_mutex;
  std::condition_variable changed;
  std::deque<Image> images;
  std::deque<Imu> imus;
  std::deque<Event> events;
  RecorderStats counters;
  bool accepting = true, stopping = false, stopping_events = false, finished = false;
  bool fault = false;
  std::string failure_reason, fingerprint;
  std::optional<std::int64_t> last_image, last_saved_image, last_imu;
  std::ofstream image_stream, imu_stream, event_stream;
  std::thread image_thread, imu_thread, event_thread;

  explicit Impl(const RecorderOptions & input) : options(input)
  {
    validate_calibration_geometry(options.calibration);
    if (
      options.output.empty() || !std::isfinite(options.image_save_fps) ||
      options.image_save_fps <= 0 || !std::isfinite(options.exposure_us) ||
      options.exposure_us <= 0 || !std::isfinite(options.duration_s) || options.duration_s <= 0 ||
      !options.image_capacity || !options.imu_capacity || !options.event_capacity)
      throw std::invalid_argument("invalid recorder options");
    if (
      options.timestamp_semantics != "host_receive_v1" &&
      options.timestamp_semantics != "daedalus_frame_v1")
      throw std::invalid_argument("unsupported timestamp semantics");
    // create_directory is atomic with respect to an existing session; never reuse a target.
    if (!std::filesystem::create_directory(options.output))
      throw std::runtime_error(
        "session output must be a fresh directory: " + options.output.string());
    std::filesystem::create_directory(options.output / "images");
    checked_write(options.output / "calibration.yaml", YAML::Dump(options.calibration));
    fingerprint = file_fingerprint(options.output / "calibration.yaml");
    write_metadata(false);
    image_stream.open(options.output / "images.csv");
    imu_stream.open(options.output / "imu.csv");
    event_stream.open(options.output / "events.csv");
    image_stream
      << "frame_id,t_image_ns,image_path,camera_frame_id,device_timestamp_ticks,exposure_us\n"
      << std::setprecision(17);
    imu_stream << "sample_id,t_imu_receive_ns,qw,qx,qy,qz,valid\n" << std::setprecision(17);
    event_stream << "t_event_ns,event_type,local_id,reason\n";
    if (!image_stream || !imu_stream || !event_stream)
      throw std::runtime_error("cannot open session CSV files");
    try {
      image_thread = std::thread([this] { save_images(); });
      imu_thread = std::thread([this] { save_imus(); });
      event_thread = std::thread([this] { save_events(); });
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = stopping_events = true;
        accepting = false;
      }
      changed.notify_all();
      if (image_thread.joinable()) image_thread.join();
      if (imu_thread.joinable()) imu_thread.join();
      if (event_thread.joinable()) event_thread.join();
      throw;
    }
  }
  std::int64_t ns(TimePoint time) const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time - options.t0).count();
  }
  void event_locked(
    std::int64_t time, const std::string & type, std::uint64_t id, const std::string & reason)
  {
    // Diagnostic event time means enqueue occurrence, not the potentially stale
    // or invalid sensor timestamp associated with its local_id.
    (void)time;
    if (events.size() >= options.event_capacity) {
      ++counters.events_dropped;
      fault = true;
      return;
    }
    events.push_back({ns(Clock::now()), type, id, reason});
    changed.notify_all();
  }
  void fail_locked(const std::string & reason)
  {
    fault = true;
    if (failure_reason.empty()) failure_reason = reason;
  }
  void io_failure(const std::string & reason, std::uint64_t id)
  {
    std::lock_guard<std::mutex> lock(mutex);
    fail_locked(reason);
    event_locked(ns(Clock::now()), "write_failure", id, reason);
  }
  void write_metadata(bool complete)
  {
    YAML::Node metadata;
    metadata["schema_version"] = 1;
    metadata["session_id"] = options.output.filename().string();
    metadata["clock"] = "steady_clock";
    metadata["timestamp_semantics"] = options.timestamp_semantics;
    metadata["t0_steady_ns"] =
      std::chrono::duration_cast<std::chrono::nanoseconds>(options.t0.time_since_epoch()).count();
    metadata["complete"] = complete;
    metadata["calibration_fingerprint"] = fingerprint;
    metadata["source_head"] = options.source_head;
    metadata["camera_name"] = options.camera_name;
    metadata["imu_source"] =
      options.camera_name == "daedalus" ? "frame_coupled_simulator" : "cboard_receive_observer";
    metadata["image_save_fps"] = options.image_save_fps;
    metadata["exposure_us"] = options.exposure_us;
    metadata["duration_s"] = options.duration_s;
    metadata["actual_duration_s"] =
      std::chrono::duration<double>(Clock::now() - options.t0).count();
    metadata["length_unit"] = "m";
    metadata["image_queue_capacity"] = options.image_capacity;
    metadata["imu_queue_capacity"] = options.imu_capacity;
    metadata["event_queue_capacity"] = options.event_capacity;
    metadata["failure_reason"] = failure_reason;
    auto counts = metadata["counters"];
#define COUNT(name) counts[#name] = counters.name
    COUNT(images_received);
    COUNT(images_accepted);
    COUNT(images_written);
    COUNT(images_dropped);
    COUNT(images_failed);
    COUNT(images_skipped);
    COUNT(camera_source_dropped);
    COUNT(imu_received);
    COUNT(imu_accepted);
    COUNT(imu_written);
    COUNT(imu_dropped);
    COUNT(imu_failed);
    COUNT(imu_invalid);
    COUNT(imu_gap_count);
    COUNT(max_imu_receive_gap_ms);
    COUNT(events_written);
    COUNT(events_dropped);
#undef COUNT
    checked_write(options.output / "session.yaml", YAML::Dump(metadata));
  }
  void save_images()
  {
    for (;;) {
      Image item;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [this] { return stopping || !images.empty(); });
        if (images.empty()) return;
        item = std::move(images.front());
        images.pop_front();
      }
      bool success = false;
      try {
        const auto relative = "images/" + std::to_string(item.id) + ".png";
        if (!cv::imwrite((options.output / relative).string(), item.pixels))
          throw std::runtime_error("PNG encoding/write failed");
        image_stream << item.id << ',' << item.time << ',' << relative << ',';
        if (item.metadata.camera_frame_id) image_stream << *item.metadata.camera_frame_id;
        image_stream << ',';
        if (item.metadata.device_timestamp_ticks)
          image_stream << *item.metadata.device_timestamp_ticks;
        image_stream << ',';
        if (item.metadata.exposure_us) image_stream << *item.metadata.exposure_us;
        image_stream << '\n';
        if (!image_stream) throw std::runtime_error("images.csv write failed");
        success = true;
      } catch (const std::exception & e) {
        io_failure(e.what(), item.id);
      }
      std::lock_guard<std::mutex> lock(mutex);
      if (success)
        ++counters.images_written;
      else
        ++counters.images_failed;
    }
  }
  void save_imus()
  {
    for (;;) {
      Imu item;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [this] { return stopping || !imus.empty(); });
        if (imus.empty()) return;
        item = imus.front();
        imus.pop_front();
      }
      imu_stream << item.id << ',' << item.time << ',' << item.q.w() << ',' << item.q.x() << ','
                 << item.q.y() << ',' << item.q.z() << ',' << (item.valid ? 1 : 0) << '\n';
      if (!imu_stream) io_failure("imu.csv write failed", item.id);
      std::lock_guard<std::mutex> lock(mutex);
      if (imu_stream)
        ++counters.imu_written;
      else
        ++counters.imu_failed;
    }
  }
  void save_events()
  {
    for (;;) {
      Event item;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [this] { return stopping_events || !events.empty(); });
        if (events.empty()) return;
        item = std::move(events.front());
        events.pop_front();
      }
      event_stream << item.time << ',' << csv_string(item.type) << ',' << item.id << ','
                   << csv_string(item.reason) << '\n';
      std::lock_guard<std::mutex> lock(mutex);
      if (event_stream)
        ++counters.events_written;
      else {
        ++counters.events_dropped;
        fail_locked("events.csv write failed");
      }
    }
  }
};

TimeSyncRecorder::TimeSyncRecorder(const RecorderOptions & options)
: impl_(std::make_unique<Impl>(options))
{
}
TimeSyncRecorder::~TimeSyncRecorder()
{
  try {
    finish(false);
  } catch (...) {
  }
}
bool TimeSyncRecorder::record_image(
  const cv::Mat & pixels, TimePoint time, const ImageMetadata & metadata)
{
  auto & p = *impl_;
  const auto stamp = p.ns(time);
  std::uint64_t id;
  {
    std::lock_guard<std::mutex> lock(p.mutex);
    if (!p.accepting) return false;
    id = p.counters.images_received++;
    const bool invalid = pixels.empty() || stamp < 0 || (p.last_image && stamp <= *p.last_image) ||
                         (metadata.exposure_us &&
                          (!std::isfinite(*metadata.exposure_us) || *metadata.exposure_us <= 0));
    if (invalid) {
      ++p.counters.images_dropped;
      p.fail_locked("invalid image");
      p.event_locked(
        stamp, "invalid_image", id, "empty image, nonmonotonic time, or invalid exposure");
      return false;
    }
    p.last_image = stamp;
    if (
      p.last_saved_image &&
      static_cast<double>(stamp - *p.last_saved_image) < 1e9 / p.options.image_save_fps) {
      ++p.counters.images_skipped;
      return false;
    }
    if (p.images.size() >= p.options.image_capacity) {
      ++p.counters.images_dropped;
      p.fail_locked("image queue overflow");
      p.event_locked(stamp, "image_queue_overflow", id, "image logging queue full");
      return false;
    }
  }
  cv::Mat owned;
  try {
    owned = pixels.clone();
  } catch (const std::exception & e) {
    std::lock_guard<std::mutex> lock(p.mutex);
    ++p.counters.images_dropped;
    p.fail_locked(e.what());
    p.event_locked(stamp, "image_copy_failure", id, e.what());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(p.mutex);
    if (!p.accepting || p.images.size() >= p.options.image_capacity) {
      ++p.counters.images_dropped;
      p.fail_locked("image queue overflow or producer raced finish");
      p.event_locked(stamp, "image_queue_overflow", id, p.failure_reason);
      return false;
    }
    p.images.push_back({id, stamp, std::move(owned), metadata});
    ++p.counters.images_accepted;
    p.last_saved_image = stamp;
  }
  p.changed.notify_all();
  return true;
}
bool TimeSyncRecorder::record_imu(const Eigen::Quaterniond & q, TimePoint time)
{
  auto & p = *impl_;
  const auto stamp = p.ns(time);
  std::lock_guard<std::mutex> lock(p.mutex);
  if (!p.accepting) return false;
  const auto id = p.counters.imu_received++;
  const auto norm = q.squaredNorm();
  const bool valid = stamp >= 0 && (!p.last_imu || stamp > *p.last_imu) && q.coeffs().allFinite() &&
                     std::isfinite(norm) && std::abs(norm - 1) <= 0.1;
  if (p.last_imu && stamp > *p.last_imu) {
    const double gap = static_cast<double>(stamp - *p.last_imu) / 1e6;
    p.counters.max_imu_receive_gap_ms = std::max(p.counters.max_imu_receive_gap_ms, gap);
    if (gap > 20) {
      ++p.counters.imu_gap_count;
      p.event_locked(stamp, "imu_receive_gap", id, "host receive gap ms: " + std::to_string(gap));
    }
  }
  if (!p.last_imu || stamp > *p.last_imu) p.last_imu = stamp;
  if (!valid) {
    ++p.counters.imu_invalid;
    p.fail_locked("invalid IMU quaternion or timestamp");
    p.event_locked(
      stamp, "invalid_imu", id, "nonfinite/nonunit quaternion or nonmonotonic timestamp");
  }
  if (p.imus.size() >= p.options.imu_capacity) {
    ++p.counters.imu_dropped;
    p.fail_locked("IMU queue overflow");
    p.event_locked(stamp, "imu_queue_overflow", id, "independent IMU logging queue full");
    return false;
  }
  p.imus.push_back({id, stamp, q, valid});
  ++p.counters.imu_accepted;
  p.changed.notify_all();
  return true;
}
void TimeSyncRecorder::record_event(
  TimePoint time, const std::string & type, std::uint64_t id, const std::string & reason)
{
  auto & p = *impl_;
  std::lock_guard<std::mutex> lock(p.mutex);
  if (!p.stopping_events && !p.finished) p.event_locked(p.ns(time), type, id, reason);
}
void TimeSyncRecorder::mark_incomplete(const std::string & reason)
{
  auto & p = *impl_;
  std::lock_guard<std::mutex> lock(p.mutex);
  if (p.finished) return;
  p.fail_locked(reason);
  p.event_locked(p.ns(Clock::now()), "session_fault", 0, reason);
}
void TimeSyncRecorder::record_source_drops(std::uint64_t count)
{
  if (!count) return;
  auto & p = *impl_;
  std::lock_guard<std::mutex> lock(p.mutex);
  if (!p.accepting) return;
  p.counters.camera_source_dropped += count;
  p.fail_locked("camera source queue overflow");
  p.event_locked(
    p.ns(Clock::now()), "camera_source_queue_overflow", p.counters.images_received,
    "source frames dropped: " + std::to_string(count));
}
RecorderStats TimeSyncRecorder::stats() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->counters;
}
bool TimeSyncRecorder::failed() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->fault;
}
RecorderStats TimeSyncRecorder::finish(bool clean)
{
  auto & p = *impl_;
  std::lock_guard<std::mutex> finish_lock(p.finish_mutex);
  {
    std::lock_guard<std::mutex> lock(p.mutex);
    if (p.finished) return p.counters;
    if (!clean) p.fail_locked("shutdown was not explicitly clean");
    p.accepting = false;
    p.stopping = true;
  }
  p.changed.notify_all();
  if (p.image_thread.joinable()) p.image_thread.join();
  if (p.imu_thread.joinable()) p.imu_thread.join();
  p.image_stream.flush();
  p.imu_stream.flush();
  if (!p.image_stream || !p.imu_stream) p.io_failure("CSV flush failed", 0);
  {
    std::lock_guard<std::mutex> lock(p.mutex);
    p.stopping_events = true;
  }
  p.changed.notify_all();
  if (p.event_thread.joinable()) p.event_thread.join();
  // Producers and event worker are stopped: this summary cannot itself be lost
  // to a saturated queue, and records exact event-loss count in the CSV as well.
  if (p.counters.events_dropped) {
    p.event_stream << p.ns(Clock::now())
                   << ",event_queue_overflow,0,\"lost events: " << p.counters.events_dropped
                   << "\"\n";
    if (p.event_stream) ++p.counters.events_written;
  }
  p.event_stream.flush();
  {
    std::lock_guard<std::mutex> lock(p.mutex);
    if (!p.event_stream) p.fail_locked("events.csv flush failed");
    p.counters.complete = clean && !p.fault &&
                          p.counters.images_accepted == p.counters.images_written &&
                          p.counters.imu_accepted == p.counters.imu_written;
    try {
      p.write_metadata(p.counters.complete);
    } catch (const std::exception & e) {
      p.fail_locked(e.what());
      p.counters.complete = false;
    }
    p.finished = true;
    return p.counters;
  }
}
}  // namespace tools::time_sync

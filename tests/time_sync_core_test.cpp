#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

#include "tools/imu_time_buffer.hpp"
#include "tools/time_offset_calibration.hpp"
#include "tools/time_sync_config.hpp"
#include "tools/time_sync_session.hpp"
// v0.51 regression suite; grouped by responsibility.

namespace buffer_checks
{

using namespace tools::time_sync;
using namespace std::chrono_literals;

namespace
{
void check(bool value, const char * reason)
{
  if (!value) throw std::runtime_error(reason);
}
template <typename F>
void rejects(F action, const char * reason)
{
  bool threw = false;
  try {
    action();
  } catch (const std::exception &) {
    threw = true;
  }
  check(threw, reason);
}
TimePoint at(double ms)
{
  return TimePoint{} +
         std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::milli>(ms));
}
Eigen::Quaterniond yaw(double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
}
void orientation(const Eigen::Quaterniond & got, const Eigen::Quaterniond & want)
{
  check(std::abs(got.norm() - 1.0) < 1e-12, "query quaternion must be normalized");
  check(got.angularDistance(want) < 1e-10, "query quaternion must follow shortest rotation");
}
void test_interpolation_and_boundaries()
{
  const std::vector<TimedImu> samples{{at(0), yaw(0)}, {at(10), yaw(1)}};
  check(query_samples({}, at(0), 20).status == ImuQueryStatus::empty, "empty buffer status");
  auto middle = query_samples(samples, at(2.5), 20);
  check(middle.valid() && middle.ratio == 0.25, "sub-millisecond query interpolation ratio");
  check(middle.left_time == at(0) && middle.right_time == at(10), "query bracket diagnostics");
  orientation(middle.q, yaw(0.25));
  for (const auto & sample : samples) {
    const auto q = query_samples(samples, sample.timestamp, 1);
    check(q.valid(), "exact endpoints do not need a neighboring sample");
    check(
      q.left_time == sample.timestamp && q.right_time == sample.timestamp,
      "exact sample has zero interpolation bracket");
    orientation(q.q, sample.q);
  }
  check(
    query_samples(samples, at(-0.1), 20).status == ImuQueryStatus::before_buffer,
    "before buffer must fail instead of extrapolating");
  check(
    query_samples(samples, at(10.1), 20).status == ImuQueryStatus::missing_right,
    "future query must fail instead of reusing last sample");
  check(
    query_samples(samples, at(5), 9.9).status == ImuQueryStatus::gap_too_large,
    "large reception gap must fail");
  check(query_samples(samples, at(5), 10).valid(), "max gap boundary inclusive");
  check(query_samples({samples.front()}, at(0), 20).valid(), "single exact sample is valid");
  auto opposite = yaw(1);
  opposite.coeffs() *= -1;
  orientation(query_samples({{at(0), yaw(1)}, {at(10), opposite}}, at(5), 20).q, yaw(1));
  orientation(
    query_samples({{at(0), yaw(3.1)}, {at(10), yaw(-3.1)}}, at(5), 20).q, yaw(std::acos(-1.0)));
  orientation(query_samples({{at(0), yaw(1e-12)}, {at(10), yaw(3e-12)}}, at(5), 20).q, yaw(2e-12));
}
void test_invalid_data_and_retention()
{
  ImuTimeBuffer buffer(2);
  check(buffer.push({at(0), yaw(0)}), "initial real sample accepted");
  check(!buffer.push({at(0), yaw(1)}), "duplicate timestamps rejected");
  check(!buffer.push({at(-1), yaw(1)}), "time reversal rejected");
  check(!buffer.push({at(1), Eigen::Quaterniond(0, 0, 0, 0)}), "zero quaternion rejected");
  check(!buffer.push({at(1), Eigen::Quaterniond(2, 0, 0, 0)}), "gross quaternion norm rejected");
  check(
    !buffer.push({at(1), Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0)}),
    "nonfinite quaternion rejected");
  check(buffer.snapshot().size() == 1, "rejected samples must not evict valid history");
  check(buffer.push({at(10), yaw(1)}) && buffer.push({at(20), yaw(2)}), "new samples accepted");
  check(buffer.snapshot().size() == 2, "buffer capacity bounded");
  check(buffer.query(at(0), 20).status == ImuQueryStatus::before_buffer, "oldest sample evicted");
  const auto first = buffer.query(at(15), 20);
  const auto second = buffer.query(at(15), 20);
  check(
    first.valid() && second.valid() && buffer.snapshot().size() == 2,
    "queries must not consume history");
  orientation(second.q, yaw(1.5));
  buffer.clear();
  check(buffer.query(at(10), 20).status == ImuQueryStatus::empty, "clear removes history");
  check(buffer.push({at(-20), yaw(0)}), "clear permits new timestamp epoch");
  rejects([] { ImuTimeBuffer invalid(0); }, "zero capacity rejected");
  const auto invalid = ImuQueryStatus::invalid_sample;
  check(
    query_samples({{at(0), yaw(0)}, {at(0), yaw(1)}}, at(0), 20).status == invalid,
    "pure query rejects duplicate data");
  check(
    query_samples({{at(10), yaw(0)}, {at(0), yaw(1)}}, at(5), 20).status == invalid,
    "pure query rejects unsorted data");
  check(
    query_samples({{at(0), Eigen::Quaterniond(0, 0, 0, 0)}}, at(0), 20).status == invalid,
    "pure query rejects invalid quaternion");
  ImuSampleIndex indexed({{at(0), yaw(0)}, {at(10), yaw(1)}});
  check(indexed.valid(), "immutable sample index accepts ordered valid samples");
  orientation(indexed.query(at(5), 20).q, yaw(0.5));
  ImuSampleIndex bad_index({{at(1), yaw(0)}, {at(0), yaw(1)}});
  check(
    !bad_index.valid() && bad_index.query(at(0), 20).status == invalid,
    "immutable index rejects malformed sequence before queries");
  check(buffer.query(at(-20), 0).status == invalid, "zero max gap rejected");
  check(buffer.wait_query(at(-20), 20, -1).status == invalid, "negative wait budget rejected");
}
void test_waiting()
{
  ImuTimeBuffer buffer;
  buffer.push({at(0), yaw(0)});
  auto result = std::async(std::launch::async, [&] { return buffer.wait_query(at(5), 20, 250); });
  std::this_thread::sleep_for(10ms);
  buffer.push({at(10), yaw(1)});
  auto q = result.get();
  check(q.valid() && q.wait_ms > 0 && q.wait_ms < 250, "right sample arrival wakes waiter");
  orientation(q.q, yaw(0.5));
  const auto start = Clock::now();
  const auto timeout = buffer.wait_query(at(30), 20, 15);
  const auto elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  check(timeout.status == ImuQueryStatus::missing_right, "wait timeout keeps failure status");
  check(elapsed >= 10 && elapsed < 200 && timeout.wait_ms >= 10, "bounded wait diagnostics");
  check(
    buffer.wait_query(at(-1), 20, 1000).wait_ms < 50,
    "unrecoverable before-buffer query returns immediately");
  ImuTimeBuffer empty;
  auto pending = std::async(std::launch::async, [&] { return empty.wait_query(at(0), 20, 250); });
  std::this_thread::sleep_for(10ms);
  empty.push({at(0), yaw(1)});
  check(pending.get().valid(), "empty buffer wait can acquire an exact sample");
}
void test_configuration()
{
  auto defaults = load_config(YAML::Load("{}"));
  check(
    defaults.imu_delay_ms == 1 && defaults.max_gap_ms == 20 && defaults.wait_ms == 12,
    "default compatible timing settings");
  check(
    load_config(YAML::Load("delay: -0.35")).imu_delay_ms == -0.35,
    "legacy signed fractional delay");
  check(
    load_config(YAML::Load("imu_delay_ms: 0.125\ndelay: 0.125")).imu_delay_ms == 0.125,
    "identical aliases allowed");
  auto config =
    load_config(YAML::Load("imu_delay_ms: -2.125\nimu_max_gap_ms: 15.5\nimu_wait_ms: 0"));
  check(
    config.imu_delay_ms == -2.125 && config.max_gap_ms == 15.5 && config.wait_ms == 0,
    "explicit settings preserve precision and zero wait");
  for (const auto * text :
       {"imu_delay_ms: 1\ndelay: 2", "imu_delay_ms: .nan", "imu_delay_ms: .inf",
        "imu_delay_ms: 1001", "imu_delay_ms: -1001", "imu_wait_ms: -1", "imu_wait_ms: 1001",
        "imu_max_gap_ms: 0", "imu_max_gap_ms: .nan", "imu_max_gap_ms: 1001", "delay: nope"}) {
    rejects([&] { load_config(YAML::Load(text)); }, "invalid or conflicting config rejected");
  }
  check(query_time(at(10), 0.125) == at(9.875), "positive delay queries earlier");
  check(query_time(at(10), -0.125) == at(10.125), "negative delay queries later");
  rejects(
    [] { query_time(at(10), std::numeric_limits<double>::infinity()); },
    "nonfinite query delay rejected");
  rejects([] { query_time(TimePoint::min(), 1); }, "query time underflow rejected");
  rejects([] { query_time(TimePoint::max(), -1); }, "query time overflow rejected");
}
}  // namespace
int run()
{
  try {
    test_interpolation_and_boundaries();
    test_invalid_data_and_retention();
    test_waiting();
    test_configuration();
    std::cout << "time_sync_buffer_test: all checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}

}  // namespace buffer_checks

namespace offset_checks
{

using namespace tools::time_sync;
static void require(bool value, const char * text)
{
  if (!value) throw std::runtime_error(text);
}
static TimePoint at(double seconds)
{
  return TimePoint(
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds)));
}
static std::vector<TimedImu> motion(bool stationary = false, bool uniform = false)
{
  std::vector<TimedImu> result;
  for (int i = 0; i <= 2600; ++i) {
    const double t = i * .01;
    const double yaw = stationary ? 0
                       : uniform  ? .4 * t
                                  : .4 * std::sin(1.9 * t) + .2 * std::sin(.37 * t);
    const double pitch = stationary || uniform ? 0 : .2 * std::sin(1.3 * t);
    result.push_back(
      {at(t), Eigen::Quaterniond(
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()))});
  }
  return result;
}
static std::vector<BoardObservation> observations(
  const std::vector<TimedImu> & imu, double delay, const CalibrationGeometry & geometry = {})
{
  std::vector<BoardObservation> frames;
  const Eigen::Vector3d fixed(3, .4, 1.7);
  for (int i = 0; i < 720; ++i) {
    const auto timestamp = at(.5 + i / 30.0);
    const auto query = query_samples(
      imu,
      timestamp - std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double, std::milli>(delay)),
      20);
    require(query.valid(), "test fixture query valid");
    const Eigen::Matrix3d R = geometry.R_gimbal2imubody.transpose() * query.q.toRotationMatrix() *
                              geometry.R_gimbal2imubody;
    const Eigen::Vector3d p =
      geometry.R_camera2gimbal.transpose() * (R.transpose() * fixed - geometry.t_camera2gimbal);
    frames.push_back({static_cast<std::uint64_t>(i), timestamp, p, .2});
  }
  return frames;
}
int run()
{
  try {
    auto imu = motion();
    CalibrationGeometry g;
    g.R_camera2gimbal = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    g.R_gimbal2imubody = Eigen::AngleAxisd(-.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
    g.t_camera2gimbal = Eigen::Vector3d(.1, -.02, .04);
    for (const double delay : {3.7, -4.3, .25}) {
      OffsetScanOptions options;
      options.step_ms = .05;
      auto result = calibrate_time_offset(observations(imu, delay, g), imu, g, options);
      require(result.status == "candidate", "varying generated motion yields candidate");
      require(
        std::abs(result.best.delay_ms - delay) <= .025001,
        "signed submillisecond offset recovered");
      require(result.best.rms_m < 1e-8, "same SLERP data has zero residual at truth");
      require(result.baseline.frame_count == result.best.frame_count, "baseline uses same frames");
      require(result.segment_best_delay_ms.size() == 3, "three contiguous segments reported");
    }
    for (const auto & degenerate : {motion(true), motion(false, true)}) {
      auto result = calibrate_time_offset(observations(degenerate, 3.7), degenerate, {});
      require(result.status == "inconclusive", "stationary and uniform rotations unidentifiable");
      require(result.reason == "flat_curve", "degeneracy exposes flat objective");
    }
    auto frames = observations(imu, 30);
    auto edge = calibrate_time_offset(frames, imu, {});
    require(
      edge.status == "inconclusive" && edge.reason == "boundary_minimum",
      "outside-range truth rejected");
    auto empty = calibrate_time_offset({}, imu, {});
    require(empty.status == "invalid_data", "empty data rejected");
    auto invalid = observations(imu, 3.7);
    for (auto & f : invalid) f.reprojection_error_px = 10;
    require(
      calibrate_time_offset(invalid, imu, {}).status == "invalid_data", "all bad PnP rejected");
    invalid = observations(imu, 3.7);
    invalid[1].timestamp = invalid[0].timestamp;
    require(
      calibrate_time_offset(invalid, imu, {}).status == "invalid_data",
      "duplicate frame time rejected");
    auto broken = imu;
    broken[4].q.coeffs().setZero();
    require(
      calibrate_time_offset(observations(imu, 3.7), broken, {}).status == "invalid_data",
      "invalid quaternion rejected");
    auto short_imu = imu;
    short_imu.erase(short_imu.begin(), short_imu.begin() + 70);
    auto common = calibrate_time_offset(observations(imu, 3.7), short_imu, {});
    require(
      common.status == "candidate" && common.common_coverage < 1,
      "edge coverage trimmed uniformly");
    for (const auto & score : common.scan)
      require(score.frame_count == common.best.frame_count, "every delay uses common frame set");
    OffsetScanOptions invalid_options;
    invalid_options.step_ms = 0;
    require(
      calibrate_time_offset(frames, imu, {}, invalid_options).status == "invalid_data",
      "invalid scan options rejected");
    auto extreme = observations(imu, 3.7);
    extreme.back().timestamp = TimePoint::max();
    require(
      calibrate_time_offset(extreme, imu, {}).reason == "timestamp_overflow",
      "timestamp overflow rejected before arithmetic");
    auto gaps = imu;
    gaps.erase(gaps.begin() + 100, gaps.begin() + 2500);
    require(
      calibrate_time_offset(observations(imu, 3.7), gaps, {}).status == "invalid_data",
      "all-gap candidates never become success");
    auto conflicting = observations(imu, -5);
    const auto zero = observations(imu, 0), plus = observations(imu, 5);
    for (std::size_t i = 240; i < 480; ++i) conflicting[i].point_camera_m = zero[i].point_camera_m;
    for (std::size_t i = 480; i < 720; ++i) conflicting[i].point_camera_m = plus[i].point_camera_m;
    require(
      calibrate_time_offset(conflicting, imu, {}).reason == "inconsistent_segments",
      "changing offset across time blocks rejected");
    auto periodic = imu;
    for (std::size_t i = 0; i < periodic.size(); ++i)
      periodic[i].q = Eigen::Quaterniond(Eigen::AngleAxisd(
        .5 * std::sin(2 * 3.141592653589793 * i * .01 / .06), Eigen::Vector3d::UnitZ()));
    OffsetScanOptions wide;
    wide.min_delay_ms = -80;
    wide.max_delay_ms = 80;
    require(
      calibrate_time_offset(observations(periodic, 3.7), periodic, {}, wide).reason ==
        "multimodal_curve",
      "periodic delay aliases rejected");
    std::cout << "time_offset_calibration_test passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

}  // namespace offset_checks

namespace session_checks
{

int run()
{
  const auto root = std::filesystem::temp_directory_path() /
                    ("time-sync-fingerprint-" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(root);
  const auto file = root / "calibration.yaml";
  {
    std::ofstream out(file, std::ios::binary);
    out << "hello";
  }
  if (tools::time_sync::file_fingerprint(file) != "fnv1a64:a430d84680aabd0b") {
    std::cerr << "fingerprint does not match known FNV-1a vector\n";
    return 1;
  }
  {
    std::ofstream out(file, std::ios::binary | std::ios::app);
    out << "!";
  }
  if (tools::time_sync::file_fingerprint(file) == "fnv1a64:a430d84680aabd0b") return 2;
  try {
    tools::time_sync::file_fingerprint(root / "missing.yaml");
    return 3;
  } catch (const std::runtime_error &) {
  }
  std::cout << "session fingerprint: passed\n";
  return 0;
}

}  // namespace session_checks

int main()
{
  if (buffer_checks::run() != 0) return 1;
  if (offset_checks::run() != 0) return 1;
  if (session_checks::run() != 0) return 1;
  return 0;
}

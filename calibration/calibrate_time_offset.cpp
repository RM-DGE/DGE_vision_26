#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>

#include "tools/time_offset_calibration.hpp"
#include "tools/time_sync_config.hpp"
#include "tools/time_sync_session.hpp"

namespace fs = std::filesystem;
using namespace tools::time_sync;
namespace
{
struct Session
{
  std::string id, semantics, fingerprint, source_head;
  CalibrationGeometry geometry;
  std::vector<TimedImu> imu;
  std::vector<BoardObservation> observations;
  std::size_t invalid_imu = 0;
  double baseline_delay_ms = 1;
};
void ensure(bool ok, const std::string & reason)
{
  if (!ok) throw std::runtime_error(reason);
}
std::vector<std::string> fields(std::string line)
{
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::vector<std::string> result;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
        field += '"';
        ++i;
      } else {
        ensure(quoted || field.empty(), "malformed CSV quote");
        quoted = !quoted;
      }
    } else if (c == ',' && !quoted) {
      result.push_back(field);
      field.clear();
    } else
      field += c;
  }
  ensure(!quoted, "malformed CSV quote");
  result.push_back(field);
  return result;
}
std::vector<std::vector<std::string>> read_csv(
  const fs::path & path, const std::vector<std::string> & header)
{
  std::ifstream input(path);
  ensure(bool(input), "cannot read " + path.filename().string());
  std::string line;
  ensure(bool(std::getline(input, line)), "missing CSV header");
  ensure(fields(line) == header, "invalid CSV header: " + path.filename().string());
  std::vector<std::vector<std::string>> rows;
  while (std::getline(input, line)) {
    ensure(!line.empty() && line != "\r", "empty CSV record: " + path.filename().string());
    auto row = fields(line);
    ensure(row.size() == header.size(), "invalid CSV column count");
    rows.push_back(std::move(row));
  }
  ensure(input.eof(), "CSV read error");
  return rows;
}
std::int64_t integer(const std::string & text)
{
  ensure(
    !text.empty() && text.front() != '-' && text.front() != '+' &&
      std::all_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; }),
    "invalid nonnegative integer");
  std::size_t size = 0;
  const auto value = std::stoll(text, &size);
  ensure(size == text.size(), "invalid integer");
  return value;
}
double number(const std::string & text)
{
  std::size_t size = 0;
  const double value = std::stod(text, &size);
  ensure(size == text.size() && std::isfinite(value), "nonfinite or invalid numeric value");
  return value;
}
TimePoint at_ns(std::int64_t ns)
{
  ensure(
    ns <= std::numeric_limits<std::int64_t>::max() - 10000000000LL, "timestamp overflow margin");
  return TimePoint(std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(ns)));
}
std::vector<double> values(const YAML::Node & yaml, const char * key, std::size_t count)
{
  const auto data = yaml[key].as<std::vector<double>>();
  ensure(data.size() == count, "invalid geometry length: " + std::string(key));
  for (double value : data) ensure(std::isfinite(value), "nonfinite geometry: " + std::string(key));
  return data;
}
Eigen::Matrix3d matrix(const YAML::Node & yaml, const char * key, bool rotation)
{
  const auto data = values(yaml, key, 9);
  Eigen::Matrix3d R = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(data.data());
  if (rotation)
    ensure(
      (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-5 &&
        std::abs(R.determinant() - 1) < 1e-5,
      "invalid rotation: " + std::string(key));
  return R;
}
fs::path image_path(const fs::path & root, const std::string & name)
{
  const fs::path path(name);
  ensure(
    !name.empty() && !path.is_absolute() && !path.has_root_name() &&
      name.find('\\') == std::string::npos && name.find(':') == std::string::npos,
    "unsafe image_path");
  for (const auto & part : path) ensure(part != ".." && part != ".", "unsafe image_path");
  ensure(path.extension() == ".png", "image_path must refer to PNG");
  const auto base = fs::weakly_canonical(root), resolved = fs::weakly_canonical(root / path);
  const auto mismatch = std::mismatch(base.begin(), base.end(), resolved.begin(), resolved.end());
  ensure(mismatch.first == base.end(), "image_path escapes session");
  return resolved;
}
Session read_session(const fs::path & root)
{
  Session session;
  const auto meta = YAML::LoadFile((root / "session.yaml").string());
  ensure(meta["schema_version"].as<int>() == 1, "unsupported schema_version");
  ensure(meta["complete"].as<bool>(), "incomplete session");
  ensure(meta["clock"].as<std::string>() == "steady_clock", "unsupported clock");
  session.id = meta["session_id"].as<std::string>();
  ensure(!session.id.empty(), "empty session_id");
  session.semantics = meta["timestamp_semantics"].as<std::string>();
  ensure(
    session.semantics == "host_receive_v1" || session.semantics == "daedalus_frame_v1",
    "unsupported timestamp_semantics");
  session.source_head = meta["source_head"].as<std::string>();
  session.fingerprint = meta["calibration_fingerprint"].as<std::string>();
  ensure(
    session.fingerprint == file_fingerprint(root / "calibration.yaml"),
    "calibration fingerprint mismatch");
  const auto yaml = YAML::LoadFile((root / "calibration.yaml").string());
  session.baseline_delay_ms = load_config(yaml).imu_delay_ms;
  session.geometry.R_camera2gimbal = matrix(yaml, "R_camera2gimbal", true);
  session.geometry.R_gimbal2imubody = matrix(yaml, "R_gimbal2imubody", true);
  const auto t = values(yaml, "t_camera2gimbal", 3);
  session.geometry.t_camera2gimbal = Eigen::Vector3d(t.data());
  const auto K = matrix(yaml, "camera_matrix", false);
  ensure(
    K(0, 0) > 0 && K(1, 1) > 0 && std::abs(K(0, 1)) < 1e-12 && std::abs(K(1, 0)) < 1e-12 &&
      std::abs(K(2, 0)) < 1e-12 && std::abs(K(2, 1)) < 1e-12 && std::abs(K(2, 2) - 1) < 1e-12,
    "invalid camera_matrix");
  const auto kd = yaml["distort_coeffs"].as<std::vector<double>>();
  ensure(
    kd.size() == 4 || kd.size() == 5 || kd.size() == 8 || kd.size() == 12 || kd.size() == 14,
    "invalid distort_coeffs length");
  for (double value : kd) ensure(std::isfinite(value), "nonfinite distort_coeffs");
  cv::Matx33d camera;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) camera(row, col) = K(row, col);
  const cv::Mat distort(kd);
  const auto board = yaml["time_sync_board"];
  const int cols = board["columns"].as<int>(), rows = board["rows"].as<int>();
  const double square = board["square_size_m"].as<double>();
  ensure(
    cols >= 2 && rows >= 2 && cols <= 100 && rows <= 100 && std::isfinite(square) && square > 0,
    "invalid time_sync_board");
  std::vector<cv::Point3f> object;
  // Put the model origin at the board centre. Reversing all detected corners
  // by 180 degrees must not change which physical point is scored.
  for (int row = 0; row < rows; ++row)
    for (int col = 0; col < cols; ++col)
      object.emplace_back(
        static_cast<float>((col - (cols - 1) * .5) * square),
        static_cast<float>((row - (rows - 1) * .5) * square), 0);
  const auto imu_rows =
    read_csv(root / "imu.csv", {"sample_id", "t_imu_receive_ns", "qw", "qx", "qy", "qz", "valid"});
  std::int64_t last_id = -1, last_time = -1;
  for (const auto & row : imu_rows) {
    const auto id = integer(row[0]), time = integer(row[1]);
    ensure(id > last_id && time > last_time, "IMU IDs/times must be strictly monotonic");
    last_id = id;
    last_time = time;
    Eigen::Quaterniond q(number(row[2]), number(row[3]), number(row[4]), number(row[5]));
    ensure(row[6] == "0" || row[6] == "1", "invalid IMU valid flag");
    if (row[6] == "0") {
      ++session.invalid_imu;
      continue;
    }
    ensure(
      std::isfinite(q.squaredNorm()) && std::abs(q.squaredNorm() - 1) <= .1,
      "invalid quaternion norm");
    session.imu.push_back({at_ns(time), q});
  }
  const auto events =
    read_csv(root / "events.csv", {"t_event_ns", "event_type", "local_id", "reason"});
  last_time = -1;
  for (const auto & row : events) {
    const auto time = integer(row[0]);
    ensure(time >= last_time, "event times must be monotonic");
    last_time = time;
    if (!row[2].empty()) integer(row[2]);
    ensure(
      row[1].find("reset") == std::string::npos && row[1].find("reconnect") == std::string::npos,
      "reference reset/reconnect requires separate session");
  }
  const auto images = read_csv(
    root / "images.csv", {"frame_id", "t_image_ns", "image_path", "camera_frame_id",
                          "device_timestamp_ticks", "exposure_us"});
  last_id = -1;
  last_time = -1;
  for (const auto & row : images) {
    const auto id = integer(row[0]), time = integer(row[1]);
    ensure(id > last_id && time > last_time, "image IDs/times must be strictly monotonic");
    last_id = id;
    last_time = time;
    for (int i : {3, 4})
      if (!row[i].empty()) integer(row[i]);
    if (!row[5].empty()) ensure(number(row[5]) >= 0, "invalid exposure_us");
    const auto path = image_path(root, row[2]);
    const cv::Mat image = cv::imread(path.string());
    ensure(!image.empty(), "unreadable image_path: " + row[2]);
    std::vector<cv::Point2f> corners;
    const int flags = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
    if (!cv::findChessboardCornersSB(image, cv::Size(cols, rows), corners, flags)) continue;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    std::vector<cv::Mat> rotations, translations;
    const int solutions = cv::solvePnPGeneric(
      object, corners, camera, distort, rotations, translations, false, cv::SOLVEPNP_IPPE);
    double best_error = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_point;
    for (int j = 0; j < solutions; ++j) {
      cv::Mat R;
      cv::Rodrigues(rotations[j], R);
      const auto & translation = translations[j];
      bool positive = true;
      for (const auto & p : object)
        if (R.at<double>(2, 0) * p.x + R.at<double>(2, 1) * p.y + translation.at<double>(2) <= 0)
          positive = false;
      if (!positive) continue;
      std::vector<cv::Point2f> projected;
      cv::projectPoints(object, rotations[j], translation, camera, distort, projected);
      double sum = 0;
      for (std::size_t k = 0; k < corners.size(); ++k) {
        const auto d = projected[k] - corners[k];
        sum += d.dot(d);
      }
      const double error = std::sqrt(sum / corners.size());
      if (std::isfinite(error) && error < best_error) {
        best_error = error;
        best_point = Eigen::Vector3d(
          translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));
      }
    }
    if (!std::isfinite(best_error) || !best_point.allFinite()) {
      continue;
    }
    session.observations.push_back(
      {static_cast<std::uint64_t>(id), at_ns(time), best_point, best_error});
  }
  const auto counters = meta["counters"];
  ensure(counters && counters.IsMap(), "missing session counters");
  const auto count = [&](const char * key) {
    ensure(counters[key].IsDefined(), std::string("missing session counter: ") + key);
    return static_cast<std::size_t>(integer(counters[key].as<std::string>()));
  };
  ensure(
    count("images_written") == images.size() && count("imu_written") == imu_rows.size() &&
      count("events_written") == events.size(),
    "session record count mismatch (truncated or modified CSV)");
  ensure(
    count("images_accepted") == images.size() && count("imu_accepted") == imu_rows.size(),
    "session accepted record count mismatch");
  for (const auto * key :
       {"images_dropped", "images_failed", "imu_dropped", "imu_failed", "imu_invalid",
        "events_dropped", "camera_source_dropped"})
    ensure(count(key) == 0, std::string("complete session has nonzero loss counter: ") + key);
  ensure(session.invalid_imu == 0, "complete session contains invalid IMU rows");
  return session;
}
YAML::Node terminal_scores(const OffsetCalibrationResult & result, const OffsetScore & candidate)
{
  YAML::Node node;
  node["common_frames"] = result.common_frames.size();
  node["common_query_coverage"] = result.common_coverage;
  if (result.baseline.valid) node["current_rms_m"] = result.baseline.rms_m;
  if (candidate.valid) node["candidate_rms_m"] = candidate.rms_m;
  return node;
}
}  // namespace
int main(int argc, char ** argv)
{
  YAML::Node summary;
  summary["status"] = "invalid_data";
  summary["offline_validation_passed"] = false;
  summary["online_verification"] = "pending_required";
  try {
    std::map<std::string, std::string> args;
    const std::vector<std::string> allowed = {
      "session",
      "validation",
      "min-delay-ms",
      "max-delay-ms",
      "step-ms",
      "max-gap-ms",
      "min-frames",
      "min-duration-s",
      "max-reprojection-px",
      "segment-tolerance-ms"};
    for (int i = 1; i < argc; ++i) {
      std::string key = argv[i];
      if (key == "--help") {
        std::cout << "calibrate_time_offset [--session DIR] [--validation INDEPENDENT_DIR]\n"
                     "  Default: data/time_sync/train_01, plus validate_01 if present.\n"
                     "  Results are printed to the terminal.\n"
                     "  --min-delay-ms -20 --max-delay-ms 20 --step-ms 0.1 --max-gap-ms 20\n"
                     "  --min-frames 300 --min-duration-s 20 --max-reprojection-px 1.5 "
                     "--segment-tolerance-ms 1\n";
        return 0;
      }
      ensure(key.rfind("--", 0) == 0, "expected --option value");
      key = key.substr(2);
      ensure(
        std::find(allowed.begin(), allowed.end(), key) != allowed.end(), "unknown option: " + key);
      ensure(i + 1 < argc && !args.count(key), "missing or duplicate option: " + key);
      args[key] = argv[++i];
    }
    if (!args.count("session")) {
      args["session"] = "data/time_sync/train_01";
      if (!args.count("validation") && fs::exists("data/time_sync/validate_01"))
        args["validation"] = "data/time_sync/validate_01";
    }
    ensure(
      fs::is_directory(args["session"]), "missing capture_time_sync session: " + args["session"]);
    OffsetScanOptions options;
    for (const auto & entry : std::vector<std::pair<std::string, double *>>{
           {"min-delay-ms", &options.min_delay_ms},
           {"max-delay-ms", &options.max_delay_ms},
           {"step-ms", &options.step_ms},
           {"max-gap-ms", &options.max_gap_ms},
           {"min-duration-s", &options.min_duration_s},
           {"max-reprojection-px", &options.max_reprojection_px},
           {"segment-tolerance-ms", &options.segment_tolerance_ms}})
      if (args.count(entry.first)) *entry.second = number(args[entry.first]);
    if (args.count("min-frames"))
      options.min_frames = static_cast<std::size_t>(integer(args["min-frames"]));
    const Session session = read_session(args["session"]);
    options.baseline_delay_ms = session.baseline_delay_ms;
    summary["current_imu_delay_ms"] = options.baseline_delay_ms;
    const auto result =
      calibrate_time_offset(session.observations, session.imu, session.geometry, options);
    summary["training"] = terminal_scores(result, result.best);
    summary["status"] = result.status;
    summary["reason"] = result.reason;
    if (result.best.valid) summary["candidate_imu_delay_ms"] = result.best.delay_ms;
    if (args.count("validation")) {
      ensure(
        fs::weakly_canonical(args["session"]) != fs::weakly_canonical(args["validation"]),
        "validation must be an independent session");
      const Session validation = read_session(args["validation"]);
      ensure(validation.id != session.id, "validation must use a distinct session_id");
      ensure(validation.semantics == session.semantics, "validation timestamp_semantics mismatch");
      ensure(
        validation.fingerprint == session.fingerprint,
        "validation calibration fingerprint mismatch");
      const auto verify = calibrate_time_offset(
        validation.observations, validation.imu, validation.geometry, options);
      const auto fixed = score_offset(
        verify.common_frames, validation.imu, validation.geometry, result.best.delay_ms,
        options.max_gap_ms);
      summary["validation"] = terminal_scores(verify, fixed);
      std::vector<double> gains;
      for (std::size_t k = 0; k < options.segment_count && !verify.common_frames.empty(); ++k) {
        auto begin =
          verify.common_frames.begin() + verify.common_frames.size() * k / options.segment_count;
        auto end = verify.common_frames.begin() +
                   verify.common_frames.size() * (k + 1) / options.segment_count;
        const std::vector<BoardObservation> segment(begin, end);
        const auto before = score_offset(
          segment, validation.imu, validation.geometry, options.baseline_delay_ms,
          options.max_gap_ms);
        const auto after = score_offset(
          segment, validation.imu, validation.geometry, result.best.delay_ms, options.max_gap_ms);
        if (before.valid && after.valid) gains.push_back(before.rms_m - after.rms_m);
      }
      double gain_mean = 0, gain_deviation = 0;
      for (double gain : gains) gain_mean += gain;
      if (!gains.empty()) gain_mean /= gains.size();
      for (double gain : gains) gain_deviation += (gain - gain_mean) * (gain - gain_mean);
      if (gains.size() > 1) gain_deviation = std::sqrt(gain_deviation / (gains.size() - 1));
      const double gain =
        fixed.valid && verify.baseline.valid ? verify.baseline.rms_m - fixed.rms_m : 0;
      const bool passed = result.status == "candidate" && verify.status == "candidate" &&
                          fixed.valid &&
                          std::abs(verify.best.delay_ms - result.best.delay_ms) <=
                            std::max(options.segment_tolerance_ms, 2 * options.step_ms) &&
                          gains.size() == options.segment_count &&
                          *std::min_element(gains.begin(), gains.end()) > options.flat_absolute_m &&
                          gain > std::max(options.flat_absolute_m, gain_deviation);
      summary["offline_validation_passed"] = passed;
      if (!passed && result.status == "candidate") {
        summary["status"] = "inconclusive";
        summary["reason"] = "independent_validation_failed";
      }
    }
    std::cout << std::setprecision(17) << summary << '\n';
    const std::string status = summary["status"].as<std::string>();
    return status == "candidate" ? 0 : status == "inconclusive" ? 3 : 2;
  } catch (const std::exception & error) {
    std::cerr << "invalid_data: " << error.what() << '\n';
    summary["status"] = "invalid_data";
    summary["reason"] = error.what();
    summary["offline_validation_passed"] = false;
    std::cout << std::setprecision(17) << summary << '\n';
    return 2;
  }
}

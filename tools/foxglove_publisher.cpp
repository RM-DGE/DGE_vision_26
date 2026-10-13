#ifdef ENABLE_FOXGLOVE
#include "tools/foxglove_publisher.hpp"

#include <foxglove/channel.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "io/command.hpp"
#include "io/talos/talos.hpp"

namespace tools
{

namespace
{
using foxglove::messages::ArrowPrimitive;
using foxglove::messages::Color;
using foxglove::messages::CubePrimitive;
using foxglove::messages::LinePrimitive;
using foxglove::messages::Point3;
using foxglove::messages::Pose;
using foxglove::messages::Quaternion;
using foxglove::messages::SceneEntity;
using foxglove::messages::SceneUpdate;
using foxglove::messages::SpherePrimitive;
using foxglove::messages::TextPrimitive;
using foxglove::messages::Vector3;

std::uint64_t now_ns()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count());
}

foxglove::messages::Timestamp to_stamp(std::uint64_t ns)
{
  return foxglove::messages::Timestamp{
    static_cast<std::uint32_t>(ns / 1000000000ull),
    static_cast<std::uint32_t>(ns % 1000000000ull)};
}

Vector3 to_vector(const Eigen::Vector3d & v) { return Vector3{v.x(), v.y(), v.z()}; }

Quaternion yaw_quaternion(double yaw)
{
  return Quaternion{0, 0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
}

foxglove::Schema telemetry_schema()
{
  static const std::string json = R"({
    "type": "object",
    "properties": {
      "fps": {"type": "number"},
      "cmd_yaw": {"type": "number"},
      "cmd_pitch": {"type": "number"},
      "cmd_yaw_vel": {"type": "number"},
      "cmd_yaw_acc": {"type": "number"},
      "cmd_pitch_vel": {"type": "number"},
      "cmd_pitch_acc": {"type": "number"},
      "target_distance": {"type": "number"},
      "control": {"type": "number"},
      "shoot": {"type": "number"}
    }
  })";
  foxglove::Schema schema;
  schema.name = "sp25.Telemetry";
  schema.encoding = "jsonschema";
  schema.data = reinterpret_cast<const std::byte *>(json.data());
  schema.data_len = json.size();
  return schema;
}

// world 参考系下的地面网格与 XYZ 三轴（X 红前 / Y 绿左 / Z 蓝上），用于坐标核对。
SceneUpdate make_reference_scene(std::uint64_t ns)
{
  SceneUpdate update;
  SceneEntity entity;
  entity.timestamp = to_stamp(ns);
  entity.frame_id = "world";
  entity.id = "reference";

  const double extent = 10.0;
  const double step = 1.0;
  LinePrimitive grid;
  grid.type = LinePrimitive::LineType::LINE_LIST;
  grid.thickness = 0.02;
  grid.color = Color{0.35, 0.35, 0.35, 1.0};
  for (double v = -extent; v <= extent + 1e-6; v += step) {
    grid.points.push_back(Point3{v, -extent, 0.0});
    grid.points.push_back(Point3{v, extent, 0.0});
    grid.points.push_back(Point3{-extent, v, 0.0});
    grid.points.push_back(Point3{extent, v, 0.0});
  }
  entity.lines.push_back(grid);

  const Color x_color{1.0, 0.0, 0.0, 1.0};
  const Color y_color{0.0, 1.0, 0.0, 1.0};
  const Color z_color{0.0, 0.5, 1.0, 1.0};
  const Quaternion x_rot{0, 0, 0, 1};
  const Quaternion y_rot{0, 0, std::sin(M_PI / 4), std::cos(M_PI / 4)};
  const Quaternion z_rot{0, -std::sin(M_PI / 4), 0, std::cos(M_PI / 4)};

  auto add_axis = [&](Quaternion rot, Color color) {
    ArrowPrimitive arrow;
    arrow.pose = Pose{};
    arrow.pose->position = Vector3{0, 0, 0};
    arrow.pose->orientation = rot;
    arrow.shaft_length = 1.0;
    arrow.shaft_diameter = 0.04;
    arrow.head_length = 0.2;
    arrow.head_diameter = 0.1;
    arrow.color = color;
    entity.arrows.push_back(arrow);
  };
  add_axis(x_rot, x_color);
  add_axis(y_rot, y_color);
  add_axis(z_rot, z_color);

  auto add_label = [&](double x, double y, double z, const char * text, Color color) {
    TextPrimitive label;
    label.pose = Pose{};
    label.pose->position = Vector3{x, y, z};
    label.billboard = true;
    label.font_size = 0.4;
    label.color = color;
    label.text = text;
    entity.texts.push_back(label);
  };
  add_label(1.25, 0, 0, "X", x_color);
  add_label(0, 1.25, 0, "Y", y_color);
  add_label(0, 0, 1.25, "Z", z_color);

  update.entities.push_back(std::move(entity));
  return update;
}
}  // namespace

struct FoxglovePublisher::Impl
{
  std::unique_ptr<foxglove::WebSocketServer> server;
  std::unique_ptr<foxglove::RawChannel> telemetry;
  std::unique_ptr<foxglove::messages::CompressedImageChannel> image;
  std::unique_ptr<foxglove::messages::SceneUpdateChannel> scene;
  std::unique_ptr<foxglove::messages::SceneUpdateChannel> reference;
  std::unique_ptr<foxglove::messages::FrameTransformChannel> tf;
  Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
  int frame_counter = 0;
  int image_period = 3;
};

FoxglovePublisher::FoxglovePublisher(std::string host, uint16_t port, std::string config_path)
: impl_(std::make_unique<Impl>())
{
  foxglove::WebSocketServerOptions options;
  options.name = "sp25_learn";
  options.host = std::move(host);
  options.port = port;

  auto server = foxglove::WebSocketServer::create(std::move(options));
  if (!server.has_value()) {
    throw std::runtime_error(
      std::string("Foxglove server failed: ") + foxglove::strerror(server.error()));
  }
  impl_->server = std::make_unique<foxglove::WebSocketServer>(std::move(server.value()));

  auto telemetry = foxglove::RawChannel::create("/sp25/telemetry", "json", telemetry_schema());
  auto image = foxglove::messages::CompressedImageChannel::create("/sp25/image/compressed");
  auto scene = foxglove::messages::SceneUpdateChannel::create("/sp25/scene");
  auto reference = foxglove::messages::SceneUpdateChannel::create("/sp25/reference_scene");
  auto tf = foxglove::messages::FrameTransformChannel::create("/tf");
  if (!telemetry.has_value() || !image.has_value() || !scene.has_value() ||
      !reference.has_value() || !tf.has_value()) {
    throw std::runtime_error("Foxglove channel creation failed");
  }
  impl_->telemetry = std::make_unique<foxglove::RawChannel>(std::move(telemetry.value()));
  impl_->image =
    std::make_unique<foxglove::messages::CompressedImageChannel>(std::move(image.value()));
  impl_->scene =
    std::make_unique<foxglove::messages::SceneUpdateChannel>(std::move(scene.value()));
  impl_->reference =
    std::make_unique<foxglove::messages::SceneUpdateChannel>(std::move(reference.value()));
  impl_->tf = std::make_unique<foxglove::messages::FrameTransformChannel>(std::move(tf.value()));

  if (!config_path.empty()) {
    try {
      const YAML::Node cfg = YAML::LoadFile(config_path);
      if (cfg["t_camera2gimbal"]) {
        const auto v = cfg["t_camera2gimbal"].as<std::vector<double>>();
        if (v.size() == 3) impl_->t_camera2gimbal = Eigen::Vector3d(v[0], v[1], v[2]);
      }
      if (cfg["R_camera2gimbal"]) {
        const auto v = cfg["R_camera2gimbal"].as<std::vector<double>>();
        if (v.size() == 9) {
          impl_->R_camera2gimbal =
            Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(v.data());
        }
      }
    } catch (const std::exception & e) {
      std::cerr << "[FOXGLOVE] config extrinsics unavailable: " << e.what() << std::endl;
    }
  }
}

FoxglovePublisher::~FoxglovePublisher() = default;

bool FoxglovePublisher::publishFrame(
  const cv::Mat & img, const io::Command & command, double target_distance, double fps,
  const io::Talos & talos, bool aim_valid, const Eigen::Vector4d & aim_xyza)
{
  const std::uint64_t source_ns = talos.last_image_timestamp_ns();
  const std::uint64_t log_ns = source_ns != 0 ? source_ns : now_ns();
  bool ok = true;

  const nlohmann::json telemetry{
    {"fps", fps},
    {"cmd_yaw", command.yaw},
    {"cmd_pitch", command.pitch},
    {"cmd_yaw_vel", command.yaw_vel},
    {"cmd_yaw_acc", command.yaw_acc},
    {"cmd_pitch_vel", command.pitch_vel},
    {"cmd_pitch_acc", command.pitch_acc},
    {"target_distance", target_distance},
    {"control", command.control ? 1 : 0},
    {"shoot", command.shoot ? 1 : 0},
  };
  const std::string telemetry_text = telemetry.dump();
  if (impl_->telemetry->log(
        reinterpret_cast<const std::byte *>(telemetry_text.data()), telemetry_text.size(),
        log_ns) != foxglove::FoxgloveError::Ok) {
    ok = false;
  }

  if (++impl_->frame_counter % impl_->image_period == 0 && !img.empty()) {
    std::vector<unsigned char> jpeg;
    if (cv::imencode(".jpg", img, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80})) {
      foxglove::messages::CompressedImage message;
      message.timestamp = to_stamp(log_ns);
      message.frame_id = "camera";
      message.format = "jpeg";
      message.data.assign(
        reinterpret_cast<const std::byte *>(jpeg.data()),
        reinterpret_cast<const std::byte *>(jpeg.data()) + jpeg.size());
      if (impl_->image->log(message, log_ns) != foxglove::FoxgloveError::Ok) ok = false;
    }
  }

  io::Poses poses;
  if (talos.read_poses(poses)) {
    foxglove::messages::FrameTransform world_gimbal;
    world_gimbal.timestamp = to_stamp(log_ns);
    world_gimbal.parent_frame_id = "world";
    world_gimbal.child_frame_id = "gimbal";
    world_gimbal.translation = to_vector(poses.odom.position);
    world_gimbal.rotation = Quaternion{
      poses.gimbal.orientation.x(), poses.gimbal.orientation.y(),
      poses.gimbal.orientation.z(), poses.gimbal.orientation.w()};
    if (impl_->tf->log(world_gimbal, log_ns) != foxglove::FoxgloveError::Ok) ok = false;

    const Eigen::Quaterniond camera_rotation(impl_->R_camera2gimbal);
    foxglove::messages::FrameTransform gimbal_camera;
    gimbal_camera.timestamp = to_stamp(log_ns);
    gimbal_camera.parent_frame_id = "gimbal";
    gimbal_camera.child_frame_id = "camera";
    gimbal_camera.translation = to_vector(impl_->t_camera2gimbal);
    gimbal_camera.rotation = Quaternion{
      camera_rotation.x(), camera_rotation.y(), camera_rotation.z(), camera_rotation.w()};
    if (impl_->tf->log(gimbal_camera, log_ns) != foxglove::FoxgloveError::Ok) ok = false;
  }

  SceneUpdate update;
  SceneEntity ground_truth;
  ground_truth.timestamp = to_stamp(log_ns);
  ground_truth.frame_id = "world";
  ground_truth.id = "ground_truth";
  for (const auto & target : talos.read_ground_truth()) {
    const Color color =
      target.team == 0 ? Color{1, 0, 0, 1} : Color{0, 0, 1, 1};
    const Vector3 center = to_vector(target.position);

    SpherePrimitive sphere;
    sphere.pose = Pose{};
    sphere.pose->position = center;
    sphere.size = Vector3{0.3, 0.3, 0.3};
    sphere.color = color;
    ground_truth.spheres.push_back(sphere);

    ArrowPrimitive arrow;
    arrow.pose = Pose{};
    arrow.pose->position = center;
    arrow.pose->orientation = yaw_quaternion(target.yaw);
    arrow.shaft_length = 0.6;
    arrow.shaft_diameter = 0.03;
    arrow.head_length = 0.12;
    arrow.head_diameter = 0.07;
    arrow.color = color;
    ground_truth.arrows.push_back(arrow);
  }
  update.entities.push_back(std::move(ground_truth));

  if (aim_valid) {
    SceneEntity aim;
    aim.timestamp = to_stamp(log_ns);
    aim.frame_id = "world";
    aim.id = "aim";
    const Vector3 center = Vector3{aim_xyza[0], aim_xyza[1], aim_xyza[2]};
    const Color magenta{1.0, 0.0, 1.0, 1.0};

    SpherePrimitive point;
    point.pose = Pose{};
    point.pose->position = center;
    point.size = Vector3{0.12, 0.12, 0.12};
    point.color = magenta;
    aim.spheres.push_back(point);

    CubePrimitive plate;
    plate.pose = Pose{};
    plate.pose->position = center;
    plate.pose->orientation = yaw_quaternion(aim_xyza[3]);
    plate.size = Vector3{0.23, 0.056, 0.02};
    plate.color = Color{1.0, 1.0, 0.0, 1.0};
    aim.cubes.push_back(plate);
    update.entities.push_back(std::move(aim));
  }
  if (impl_->scene->log(update, log_ns) != foxglove::FoxgloveError::Ok) ok = false;

  if (impl_->reference->log(make_reference_scene(log_ns), log_ns) != foxglove::FoxgloveError::Ok) {
    ok = false;
  }

  return ok;
}

}  // namespace tools
#endif  // ENABLE_FOXGLOVE

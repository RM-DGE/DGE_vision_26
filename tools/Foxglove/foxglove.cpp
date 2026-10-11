#include "foxglove.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>
#include <utility>
#include <vector>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace tools
{
using namespace foxglove::messages;

namespace
{
uint64_t now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count());
}

// 纳秒 -> Foxglove 的 sec/nsec 时间戳
Timestamp to_timestamp(uint64_t ns)
{
  Timestamp t;
  t.sec = static_cast<uint32_t>(ns / 1000000000ULL);
  t.nsec = static_cast<uint32_t>(ns % 1000000000ULL);
  return t;
}

Quaternion make_quaternion(const Eigen::Quaterniond & q)
{
  return Quaternion{q.x(), q.y(), q.z(), q.w()};
}

// 绕 Z 轴 yaw 的位姿（DGE 世界系：x 前、y 左、z 上）
Pose make_pose(double x, double y, double z, double yaw)
{
  Pose pose;
  pose.position = Vector3{x, y, z};
  const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  pose.orientation = make_quaternion(q);
  return pose;
}

Vector3 make_vector3(double x, double y, double z) { return Vector3{x, y, z}; }

Color make_color(double r, double g, double b, double a = 1.0) { return Color{r, g, b, a}; }

const Color kDetectedRed = make_color(1.0, 0.15, 0.15);
const Color kDetectedBlue = make_color(0.2, 0.5, 1.0);
const Color kDetectedOther = make_color(0.6, 0.6, 0.6);
const Color kTracked = make_color(1.0, 0.85, 0.1);
const Color kAim = make_color(0.1, 1.0, 0.3);

// 装甲板棋盘尺寸（米），局部 x = 法线（厚度）、y = 宽度、z = 高度
Vector3 armor_size(auto_aim::ArmorType type)
{
  const double width = (type == auto_aim::big) ? 0.230 : 0.135;
  return make_vector3(0.02, width, 0.125);
}

Color armor_color(auto_aim::Color color)
{
  switch (color) {
  case auto_aim::red: return kDetectedRed;
  case auto_aim::blue: return kDetectedBlue;
  default: return kDetectedOther;
  }
}
}  // namespace

struct FoxgloveServer::Impl
{
  foxglove::Context context;
  std::optional<foxglove::WebSocketServer> server;
  std::optional<CompressedImageChannel> image_ch;
  std::optional<ImageAnnotationsChannel> annotations_ch;
  std::optional<foxglove::RawChannel> debug_ch;
  std::optional<CameraCalibrationChannel> camera_info_ch;
  std::optional<FrameTransformsChannel> tf_ch;
  std::optional<SceneUpdateChannel> scene_ch;

  uint64_t last_image_ns = 0;

  // 相机外参：gimbal_link -> camera_optical_frame
  Eigen::Vector3d camera_translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond camera_rotation = Eigen::Quaterniond::Identity();
};

FoxgloveServer::FoxgloveServer(int port, std::string name) : impl_(std::make_unique<Impl>())
{
  impl_->context = foxglove::Context::create();

  foxglove::WebSocketServerOptions opts;
  opts.context = impl_->context;
  opts.name = std::move(name);
  opts.host = "0.0.0.0";
  opts.port = static_cast<uint16_t>(port);

  auto server = foxglove::WebSocketServer::create(std::move(opts));
  if (!server) {
    logger()->error("foxglove: WebSocketServer: {}", foxglove::strerror(server.error()));
    return;
  }
  impl_->server = std::move(*server);

  if (auto ch = CompressedImageChannel::create("/camera/image", impl_->context); ch) {
    impl_->image_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /camera/image: {}", foxglove::strerror(ch.error()));
  }

  if (auto ch = ImageAnnotationsChannel::create("/camera/annotations", impl_->context); ch) {
    impl_->annotations_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /camera/annotations: {}", foxglove::strerror(ch.error()));
  }

  if (auto ch = foxglove::RawChannel::create("/dge/debug", "json", std::nullopt, impl_->context);
      ch) {
    impl_->debug_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /dge/debug: {}", foxglove::strerror(ch.error()));
  }

  if (auto ch = CameraCalibrationChannel::create("/camera_info", impl_->context); ch) {
    impl_->camera_info_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /camera_info: {}", foxglove::strerror(ch.error()));
  }

  if (auto ch = FrameTransformsChannel::create("/tf", impl_->context); ch) {
    impl_->tf_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /tf: {}", foxglove::strerror(ch.error()));
  }

  if (auto ch = SceneUpdateChannel::create("/dge/scene", impl_->context); ch) {
    impl_->scene_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /dge/scene: {}", foxglove::strerror(ch.error()));
  }

  logger()->info("foxglove: WebSocket server on ws://0.0.0.0:{}", port);
}

FoxgloveServer::~FoxgloveServer()
{
  if (impl_ && impl_->server) impl_->server->stop();
}

bool FoxgloveServer::has_clients() const
{
  return impl_ && impl_->server && impl_->server->clientCount() > 0;
}

void FoxgloveServer::configure_camera(const std::string & config_path)
{
  if (!impl_->camera_info_ch) return;

  auto yaml = YAML::LoadFile(config_path);

  const auto camera_matrix = yaml["camera_matrix"].as<std::vector<double>>();
  const auto distort = yaml["distort_coeffs"].as<std::vector<double>>();
  const auto r_camera2gimbal = yaml["R_camera2gimbal"].as<std::vector<double>>();
  const auto t_camera2gimbal = yaml["t_camera2gimbal"].as<std::vector<double>>();

  const double fx = camera_matrix[0];
  const double fy = camera_matrix[4];
  const double cx = camera_matrix[2];
  const double cy = camera_matrix[5];

  // 外参：camera_optical_frame 在 gimbal_link 下的位姿
  impl_->camera_rotation = Eigen::Quaterniond(
    Eigen::Matrix3d(
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(r_camera2gimbal.data())));
  impl_->camera_translation = Eigen::Vector3d(
    t_camera2gimbal[0], t_camera2gimbal[1], t_camera2gimbal[2]);

  const uint64_t ns = now_ns();
  CameraCalibration calib;
  calib.timestamp = to_timestamp(ns);
  calib.frame_id = "camera_optical_frame";
  calib.width = 1440;
  calib.height = 1080;
  calib.distortion_model = "plumb_bob";
  calib.d = distort;
  calib.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  calib.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  calib.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  impl_->camera_info_ch->log(calib, ns);

  logger()->info(
    "foxglove: camera fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}, extrinsics t=({:.3f},{:.3f},{:.3f})",
    fx, fy, cx, cy, impl_->camera_translation.x(), impl_->camera_translation.y(),
    impl_->camera_translation.z());
}

void FoxgloveServer::publish_tf(const Eigen::Quaterniond & gimbal_world_q)
{
  if (!impl_->tf_ch || !has_clients()) return;

  const uint64_t ns = now_ns();
  FrameTransforms msg;

  {
    FrameTransform tf;
    tf.timestamp = to_timestamp(ns);
    tf.parent_frame_id = "odom";
    tf.child_frame_id = "gimbal_link";
    tf.translation = make_vector3(0.0, 0.0, 0.0);
    tf.rotation = make_quaternion(gimbal_world_q);
    msg.transforms.push_back(tf);
  }
  {
    FrameTransform tf;
    tf.timestamp = to_timestamp(ns);
    tf.parent_frame_id = "gimbal_link";
    tf.child_frame_id = "camera_optical_frame";
    tf.translation = make_vector3(
      impl_->camera_translation.x(), impl_->camera_translation.y(),
      impl_->camera_translation.z());
    tf.rotation = make_quaternion(impl_->camera_rotation);
    msg.transforms.push_back(tf);
  }

  impl_->tf_ch->log(msg, ns);
}

void FoxgloveServer::publish_scene(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
  bool aim_valid, const Eigen::Vector4d & aim_xyza)
{
  if (!impl_->scene_ch || !has_clients()) return;

  const uint64_t ns = now_ns();
  SceneUpdate update;

  // 检测到的装甲板
  {
    SceneEntity entity;
    entity.timestamp = to_timestamp(ns);
    entity.frame_id = "odom";
    entity.id = "detections";
    entity.frame_locked = false;

    for (const auto & armor : armors) {
      CubePrimitive cube;
      cube.pose = make_pose(
        armor.xyz_in_world.x(), armor.xyz_in_world.y(), armor.xyz_in_world.z(),
        armor.ypr_in_world[0]);
      cube.size = armor_size(armor.type);
      cube.color = armor_color(armor.color);
      entity.cubes.push_back(cube);
    }
    update.entities.push_back(std::move(entity));
  }

  // 跟踪目标：预测的各装甲板 + 中心
  {
    SceneEntity entity;
    entity.timestamp = to_timestamp(ns);
    entity.frame_id = "odom";
    entity.id = "tracked_target";
    entity.frame_locked = false;

    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto xyza_list = target.armor_xyza_list();

      double cx = 0.0, cy = 0.0, cz = 0.0;
      for (const auto & xyza : xyza_list) {
        CubePrimitive cube;
        cube.pose = make_pose(xyza[0], xyza[1], xyza[2], xyza[3]);
        cube.size = armor_size(target.armor_type);
        cube.color = make_color(kTracked.r, kTracked.g, kTracked.b, 0.45);
        entity.cubes.push_back(cube);

        cx += xyza[0];
        cy += xyza[1];
        cz += xyza[2];
      }

      if (!xyza_list.empty()) {
        const double n = static_cast<double>(xyza_list.size());
        cx /= n;
        cy /= n;
        cz /= n;

        SpherePrimitive center;
        center.pose = make_pose(cx, cy, cz, 0.0);
        center.size = make_vector3(0.12, 0.12, 0.12);
        center.color = make_color(kTracked.r, kTracked.g, kTracked.b, 0.6);
        entity.spheres.push_back(center);
      }
    }
    update.entities.push_back(std::move(entity));
  }

  // 瞄准点 + 指向线
  {
    SceneEntity entity;
    entity.timestamp = to_timestamp(ns);
    entity.frame_id = "odom";
    entity.id = "aiming_point";
    entity.frame_locked = false;

    if (aim_valid) {
      SpherePrimitive point;
      point.pose = make_pose(aim_xyza[0], aim_xyza[1], aim_xyza[2], aim_xyza[3]);
      point.size = make_vector3(0.06, 0.06, 0.06);
      point.color = kAim;
      entity.spheres.push_back(point);

      LinePrimitive line;
      line.type = LinePrimitive::LineType::LINE_STRIP;
      line.thickness = 0.01;
      line.color = make_color(kAim.r, kAim.g, kAim.b, 0.5);
      Point3 origin;
      origin.x = 0.0;
      origin.y = 0.0;
      origin.z = 0.0;
      Point3 tip;
      tip.x = aim_xyza[0];
      tip.y = aim_xyza[1];
      tip.z = aim_xyza[2];
      line.points.push_back(origin);
      line.points.push_back(tip);
      entity.lines.push_back(line);
    }
    update.entities.push_back(std::move(entity));
  }

  impl_->scene_ch->log(update, ns);
}

void FoxgloveServer::publish_image(const cv::Mat & bgr, int quality)
{
  if (bgr.empty() || !impl_->image_ch) return;
  if (!has_clients()) return;

  // 限频约 15fps，避免占满带宽
  const uint64_t ns = now_ns();
  if (ns - impl_->last_image_ns < 66000000ULL) return;
  impl_->last_image_ns = ns;

  std::vector<uint8_t> buf;
  std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality};
  cv::imencode(".jpg", bgr, buf, params);

  CompressedImage msg;
  msg.timestamp = to_timestamp(ns);
  msg.frame_id = "camera_optical_frame";
  msg.format = "jpeg";
  msg.data.resize(buf.size());
  std::memcpy(msg.data.data(), buf.data(), buf.size());
  impl_->image_ch->log(msg, ns);
}

void FoxgloveServer::publish_annotations(const std::list<auto_aim::Armor> & armors)
{
  if (!impl_->annotations_ch || !has_clients()) return;

  const uint64_t ns = now_ns();
  ImageAnnotations ann;
  ann.timestamp = to_timestamp(ns);

  for (const auto & a : armors) {
    if (a.points.size() < 4) continue;

    PointsAnnotation pa;
    pa.timestamp = to_timestamp(ns);
    pa.type = PointsAnnotation::PointsAnnotationType::LINE_LOOP;
    pa.points.reserve(4);
    for (int i = 0; i < 4; ++i)
      pa.points.push_back(Point2{
        static_cast<double>(a.points[i].x), static_cast<double>(a.points[i].y)});

    // 按敌方颜色上色：蓝方蓝框、红方红框、其它绿框
    Color color;
    if (a.color == auto_aim::blue) {
      color = {0.0, 0.4, 1.0, 1.0};
    } else if (a.color == auto_aim::red) {
      color = {1.0, 0.0, 0.0, 1.0};
    } else {
      color = {0.0, 1.0, 0.0, 1.0};
    }
    pa.outline_color = color;
    pa.fill_color = Color{color.r, color.g, color.b, 0.0};
    pa.thickness = 2.0;
    ann.points.push_back(std::move(pa));
  }

  impl_->annotations_ch->log(ann, ns);
}

void FoxgloveServer::publish_debug(
  double fps, const io::Command & command, const std::list<auto_aim::Target> & targets,
  const std::list<auto_aim::Armor> & armors, bool aim_valid, const Eigen::Vector4d & aim_xyza)
{
  if (!impl_->debug_ch || !has_clients()) return;

  // 曲线字段，Plot 面板可直接选
  nlohmann::json dbg;
  dbg["fps"] = fps;
  dbg["yaw"] = command.yaw;
  dbg["pitch"] = command.pitch;
  dbg["shoot"] = command.shoot ? 1 : 0;
  dbg["armors"] = static_cast<int>(armors.size());
  dbg["targets"] = static_cast<int>(targets.size());
  if (!targets.empty()) {
    auto x = targets.front().ekf_x();
    dbg["ekf"] = std::vector<double>(x.data(), x.data() + x.size());
  }
  if (aim_valid) {
    dbg["aim_x"] = aim_xyza[0];
    dbg["aim_y"] = aim_xyza[1];
    dbg["aim_z"] = aim_xyza[2];
    dbg["aim_yaw"] = aim_xyza[3];
  }

  const std::string data = dbg.dump();
  impl_->debug_ch->log(reinterpret_cast<const std::byte *>(data.data()), data.size(), now_ns());
}

}  // namespace tools

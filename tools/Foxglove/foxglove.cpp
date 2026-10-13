#include "foxglove.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

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

Vector3 make_vector3(double x, double y, double z) { return Vector3{x, y, z}; }

// 绕 Z 轴 yaw 的位姿（DGE 世界系：x 前、y 左、z 上）
Pose make_pose(double x, double y, double z, double yaw)
{
  Pose pose;
  pose.position = make_vector3(x, y, z);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  pose.orientation = make_quaternion(q);
  return pose;
}

// yaw + pitch（Rz*Ry，法线沿局部 +x），与 Solver::reproject_armor 的板面约定一致
Pose make_pose_ypr(double x, double y, double z, double yaw, double pitch)
{
  Pose pose;
  pose.position = make_vector3(x, y, z);
  const Eigen::Quaterniond q =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
  pose.orientation = make_quaternion(q);
  return pose;
}

// Solver 的板面俯仰约定：outpost -15°，其余 +15°
double armor_pitch(auto_aim::ArmorName name)
{
  return (name == auto_aim::outpost) ? -15.0 * M_PI / 180.0 : 15.0 * M_PI / 180.0;
}

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

// 指向任意方向的箭头：pose 在箭尾，+x 旋转到 (tip - tail)
ArrowPrimitive make_arrow(
  const Eigen::Vector3d & tail, const Eigen::Vector3d & tip, double shaft_diameter,
  const Color & color)
{
  const Eigen::Vector3d d = tip - tail;
  const double len = d.norm();

  ArrowPrimitive arrow;
  Pose pose;
  pose.position = make_vector3(tail.x(), tail.y(), tail.z());
  if (len > 1e-6) {
    const Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), d.normalized());
    pose.orientation = make_quaternion(q);
  } else {
    pose.orientation = Quaternion{0.0, 0.0, 0.0, 1.0};
  }
  arrow.pose = pose;
  arrow.shaft_length = len;
  arrow.shaft_diameter = shaft_diameter;
  arrow.head_length = std::min(0.12, len * 0.3);
  arrow.head_diameter = shaft_diameter * 2.5;
  arrow.color = color;
  return arrow;
}

// 目标旋转中心（x[0],x[2],x[4]）+ 高度差中点
Eigen::Vector3d target_center(const Eigen::VectorXd & x)
{
  return Eigen::Vector3d(x[0], x[2], x[4] + x[10] / 2.0);
}

// 当前离云台最近的那块装甲板中心（瞄准器实际跟随的装甲板）
Eigen::Vector3d nearest_armor_xyz(const auto_aim::Target & target)
{
  Eigen::Vector3d best = Eigen::Vector3d::Zero();
  double best_dist = 1e18;
  for (const auto & xyza : target.armor_xyza_list()) {
    const double dist = xyza.head<2>().norm();
    if (dist < best_dist) {
      best_dist = dist;
      best = xyza.head<3>();
    }
  }
  return best;
}


// ---------------------------------------------------------------------------
// ROS2 CDR (XCDR1, little-endian, 无 encapsulation header) 序列化。
// 与 ros2_foxglove_bridge 的 "cdr" 编码一致，供 Foxglove Studio 3D 面板
// 按 visualization_msgs/msg/MarkerArray 渲染。
// ---------------------------------------------------------------------------
class CdrWriter
{
public:
  // CDR 流以 4 字节 RTPS encapsulation 头开始（PLAIN_CDR, little-endian）：
  // 与 @foxglove/cdr / foxglove_bridge 输出一致，否则 Studio 无法解码。
  CdrWriter()
  {
    buf_ = {std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}};
  }

  void u32(uint32_t v) { align(4); append(&v, sizeof v); }
  void i32(int32_t v) { align(4); append(&v, sizeof v); }
  void f32(float v) { align(4); append(&v, sizeof v); }
  void f64(double v) { align(8); append(&v, sizeof v); }
  void u8(uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }

  void str(const std::string & s)
  {
    align(4);
    const uint32_t len = static_cast<uint32_t>(s.size()) + 1;  // 含结尾 '\0'
    append(&len, sizeof len);
    append(s.data(), s.size());
    buf_.push_back(std::byte{0});
  }

  // 序列头：4 字节长度；元素按 element_alignment 对齐（首元素）
  void seq_begin(uint32_t count, size_t element_alignment)
  {
    align(4);
    append(&count, sizeof count);
    if (count > 0) align(element_alignment);
  }

  std::vector<std::byte> & buf() { return buf_; }
  const std::vector<std::byte> & buf() const { return buf_; }

private:
  // 对齐基准是 CDR 数据起点（跳过 4 字节封装头），与 @foxglove/cdr 的 origin 一致
  static constexpr size_t kHeaderSize = 4;

  void align(size_t n)
  {
    const size_t rel = buf_.size() - kHeaderSize;
    const size_t r = rel % n;
    if (r != 0) buf_.resize(buf_.size() + (n - r), std::byte{0});
  }
  void append(const void * p, size_t n)
  {
    const auto * b = static_cast<const std::byte *>(p);
    buf_.insert(buf_.end(), b, b + n);
  }
  std::vector<std::byte> buf_;
};

// visualization_msgs/msg/Marker 的字段子集（按字段顺序）
struct RosMarker
{
  std::string ns;
  int32_t id = 0;
  int32_t type = 0;    // ARROW=0 CUBE=1 SPHERE=2 TEXT_VIEW_FACING=9
  int32_t action = 0;  // ADD/MODIFY=0
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  std::array<float, 4> color = {1.0f, 1.0f, 1.0f, 1.0f};
  uint32_t lifetime_ns = 0;
  bool frame_locked = false;
  std::vector<Eigen::Vector3d> points;
  std::string text;
};

// 按 humble 版 Marker.msg 字段顺序序列化单个 Marker
void write_marker(
  CdrWriter & w, const RosMarker & m, uint32_t sec, uint32_t nanosec, const std::string & frame_id)
{
  // std_msgs/Header
  w.i32(static_cast<int32_t>(sec));
  w.u32(nanosec);
  w.str(frame_id);
  w.str(m.ns);
  w.i32(m.id);
  w.i32(m.type);
  w.i32(m.action);
  // geometry_msgs/Pose
  w.f64(m.position.x());
  w.f64(m.position.y());
  w.f64(m.position.z());
  w.f64(m.orientation.x());
  w.f64(m.orientation.y());
  w.f64(m.orientation.z());
  w.f64(m.orientation.w());
  // geometry_msgs/Vector3 scale
  w.f64(m.scale.x());
  w.f64(m.scale.y());
  w.f64(m.scale.z());
  // std_msgs/ColorRGBA
  for (const float c : m.color) w.f32(c);
  // builtin_interfaces/Duration lifetime
  w.i32(0);
  w.u32(m.lifetime_ns);
  // bool frame_locked
  w.u8(m.frame_locked ? 1 : 0);
  // geometry_msgs/Point[] points
  w.seq_begin(static_cast<uint32_t>(m.points.size()), 8);
  for (const auto & p : m.points) {
    w.f64(p.x());
    w.f64(p.y());
    w.f64(p.z());
  }
  // std_msgs/ColorRGBA[] colors（空）
  w.seq_begin(0, 4);
  // string texture_resource
  w.str("");
  // sensor_msgs/CompressedImage texture（空：header + format + data）
  w.i32(static_cast<int32_t>(sec));
  w.u32(nanosec);
  w.str(frame_id);
  w.str("");
  w.seq_begin(0, 1);
  // UVCoordinate[] uv_coordinates（空）
  w.seq_begin(0, 4);
  // string text
  w.str(m.text);
  // string mesh_resource
  w.str("");
  // MeshFile mesh_file（空：filename + data）
  w.str("");
  w.seq_begin(0, 1);
  // bool mesh_use_embedded_materials
  w.u8(0);
}

// ros2msg 格式的完整消息定义文本（visualization_msgs/msg/MarkerArray）
const std::string & marker_array_schema()
{
  static const std::string schema = [] {
    const std::string sep = std::string(80, '=') + "\nMSG: ";
    std::string s;
    const auto add = [&](const std::string & name, const std::string & text) {
      if (s.empty()) { s = text; }
      else { s += "\n" + sep + name + "\n" + text; }
    };
    add("visualization_msgs/msg/MarkerArray", "Marker[] markers\n");
    add(
      "visualization_msgs/Marker", R"(
std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
builtin_interfaces/Duration lifetime
bool frame_locked
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] colors
string texture_resource
sensor_msgs/CompressedImage texture
UVCoordinate[] uv_coordinates
string text
string mesh_resource
MeshFile mesh_file
bool mesh_use_embedded_materials
)");
    add(
      "std_msgs/Header", R"(
builtin_interfaces/Time stamp
string frame_id
)");
    add("std_msgs/ColorRGBA", "float32 r\nfloat32 g\nfloat32 b\nfloat32 a\n");
    add("geometry_msgs/Pose", "Point position\nQuaternion orientation\n");
    add("geometry_msgs/Point", "float64 x\nfloat64 y\nfloat64 z\n");
    add("geometry_msgs/Quaternion", "float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n");
    add("geometry_msgs/Vector3", "float64 x\nfloat64 y\nfloat64 z\n");
    add("builtin_interfaces/Time", "int32 sec\nuint32 nanosec\n");
    add("builtin_interfaces/Duration", "int32 sec\nuint32 nanosec\n");
    add(
      "sensor_msgs/CompressedImage", R"(
std_msgs/Header header
string format
uint8[] data
)");
    add("visualization_msgs/UVCoordinate", "float32 u\nfloat32 v\n");
    add("visualization_msgs/MeshFile", "string filename\nuint8[] data\n");
    return s;
  }();
  return schema;
}

// ros2msg 格式：geometry_msgs/msg/PointStamped（/aiming_point）
const std::string & point_stamped_schema()
{
  static const std::string schema = [] {
    const std::string sep = std::string(80, '=') + "\nMSG: ";
    std::string s = "std_msgs/Header header\ngeometry_msgs/Point point\n";
    s += "\n" + sep + "std_msgs/Header" + "\n" + "builtin_interfaces/Time stamp\nstring frame_id\n";
    s += "\n" + sep + "builtin_interfaces/Time" + "\n" + "int32 sec\nuint32 nanosec\n";
    s += "\n" + sep + "geometry_msgs/Point" + "\n" + "float64 x\nfloat64 y\nfloat64 z\n";
    return s;
  }();
  return schema;
}

foxglove::Schema cdr_schema(const std::string & type_name, const std::string & text)
{
  return foxglove::Schema{
    type_name, "ros2msg", reinterpret_cast<const std::byte *>(text.data()), text.size()};
}

void log_raw(
  std::optional<foxglove::RawChannel> & ch, const std::vector<std::byte> & data, uint64_t ns)
{
  if (ch) ch->log(data.data(), data.size(), ns);
}

// YAML 标量 -> bool/int64/double/string（yaml-cpp 不解析隐式 tag，需自行判断）
foxglove::ParameterValue yaml_scalar(const std::string & s)
{
  if (s == "true" || s == "True" || s == "TRUE") return foxglove::ParameterValue(true);
  if (s == "false" || s == "False" || s == "FALSE") return foxglove::ParameterValue(false);

  if (!s.empty()) {
    char * end = nullptr;
    errno = 0;
    const long long i = std::strtoll(s.c_str(), &end, 10);
    if (end && *end == '\0' && errno == 0) return foxglove::ParameterValue(static_cast<int64_t>(i));

    end = nullptr;
    errno = 0;
    const double d = std::strtod(s.c_str(), &end);
    if (end && *end == '\0' && end != s.c_str() && errno == 0)
      return foxglove::ParameterValue(d);
  }
  return foxglove::ParameterValue(s);
}

// YAML 配置值 -> Foxglove 参数值（Parameters 面板）
foxglove::ParameterValue yaml_to_param(const YAML::Node & node)
{
  if (node.IsSequence()) {
    std::vector<foxglove::ParameterValue> arr;
    for (const auto & e : node) arr.push_back(yaml_to_param(e));
    return foxglove::ParameterValue(std::move(arr));
  }
  if (node.IsMap()) {
    std::map<std::string, foxglove::ParameterValue> dict;
    for (const auto & kv : node)
      dict.emplace(kv.first.as<std::string>(), yaml_to_param(kv.second));
    return foxglove::ParameterValue(std::move(dict));
  }
  return yaml_scalar(node.Scalar());
}
}  // namespace

struct FoxgloveServer::Impl
{
  foxglove::Context context;
  std::optional<foxglove::WebSocketServer> server;
  std::optional<CompressedImageChannel> image_ch;
  std::optional<CompressedImageChannel> result_img_ch;   // /detector/result_img/compressed
  std::optional<CompressedImageChannel> number_img_ch;   // /detector/number_img
  std::optional<ImageAnnotationsChannel> annotations_ch;
  std::optional<foxglove::RawChannel> debug_ch;
  std::optional<CameraCalibrationChannel> camera_info_ch;
  std::optional<FrameTransformsChannel> tf_ch;
  std::optional<SceneUpdateChannel> scene_ch;
  std::optional<SceneUpdateChannel> prediction_ch;       // /dge/prediction
  std::optional<foxglove::RawChannel> rm_target_ch;
  std::optional<foxglove::RawChannel> rm_info_ch;
  std::optional<foxglove::RawChannel> rm_armors_ch;
  std::optional<foxglove::RawChannel> detector_marker_ch;  // /detector/marker (cdr MarkerArray)
  std::optional<foxglove::RawChannel> tracker_marker_ch;   // /tracker/marker (cdr MarkerArray)
  std::optional<foxglove::RawChannel> aiming_point_ch;     // /aiming_point (cdr PointStamped)
  std::vector<foxglove::Parameter> parameters;             // Parameters 面板

  uint64_t last_image_ns = 0;

  // 上一帧跟踪状态（/tracker/info 的抖动指标 position_diff / yaw_diff）
  bool has_last_target = false;
  Eigen::Vector3d last_center = Eigen::Vector3d::Zero();
  double last_yaw = 0.0;

  // 相机外参：gimbal_link -> camera_optical_frame
  Eigen::Vector3d camera_translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond camera_rotation = Eigen::Quaterniond::Identity();
  std::optional<CameraCalibration> camera_calibration;
};

FoxgloveServer::FoxgloveServer(int port, std::string name) : impl_(std::make_unique<Impl>())
{
  impl_->context = foxglove::Context::create();

  foxglove::WebSocketServerOptions opts;
  opts.context = impl_->context;
  opts.name = std::move(name);
  opts.host = "0.0.0.0";
  opts.port = static_cast<uint16_t>(port);
  opts.capabilities = foxglove::WebSocketServerCapabilities::Parameters;
  opts.supported_encodings = {"cdr", "json", "protobuf"};

  // Parameters 面板：返回 yaml 里的参数；客户端修改时同步镜像
  opts.callbacks.onGetParameters =
    [this](
      uint32_t, std::optional<std::string_view>,
      const std::vector<std::string_view> & names) -> std::vector<foxglove::Parameter> {
    std::vector<foxglove::Parameter> out;
    for (const auto & p : impl_->parameters) out.push_back(p.clone());
    return out;
  };
  opts.callbacks.onSetParameters =
    [this](
      uint32_t, std::optional<std::string_view>,
      const std::vector<foxglove::ParameterView> & params) -> std::vector<foxglove::Parameter> {
    for (const auto & view : params) {
      auto updated = view.clone();
      bool found = false;
      for (auto & stored : impl_->parameters) {
        if (stored.name() == updated.name()) {
          stored = std::move(updated);
          found = true;
          break;
        }
      }
      if (!found) impl_->parameters.push_back(std::move(updated));
    }
    std::vector<foxglove::Parameter> out;
    for (const auto & p : impl_->parameters) out.push_back(p.clone());
    return out;
  };

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

  if (auto ch = CompressedImageChannel::create("/detector/result_img/compressed", impl_->context);
      ch) {
    impl_->result_img_ch = std::move(*ch);
  } else {
    logger()->error(
      "foxglove: create /detector/result_img/compressed: {}", foxglove::strerror(ch.error()));
  }

  if (auto ch = CompressedImageChannel::create("/detector/number_img", impl_->context); ch) {
    impl_->number_img_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /detector/number_img: {}", foxglove::strerror(ch.error()));
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

  if (auto ch = SceneUpdateChannel::create("/dge/prediction", impl_->context); ch) {
    impl_->prediction_ch = std::move(*ch);
  } else {
    logger()->error("foxglove: create /dge/prediction: {}", foxglove::strerror(ch.error()));
  }

  const std::string & object_schema_text = []() -> const std::string & {
    static const std::string s = R"({"type":"object"})";
    return s;
  }();
  const auto json_schema = [&object_schema_text](const char * name) {
    return foxglove::Schema{
      name, "jsonschema", reinterpret_cast<const std::byte *>(object_schema_text.data()),
      object_schema_text.size()};
  };
  if (auto ch = foxglove::RawChannel::create(
        "/tracker/target", "json", json_schema("auto_aim_interfaces/msg/Target"),
        impl_->context); ch) impl_->rm_target_ch = std::move(*ch);
  if (auto ch = foxglove::RawChannel::create(
        "/tracker/info", "json", json_schema("auto_aim_interfaces/msg/TrackerInfo"),
        impl_->context); ch) impl_->rm_info_ch = std::move(*ch);
  if (auto ch = foxglove::RawChannel::create(
        "/detector/armors", "json", json_schema("auto_aim_interfaces/msg/Armors"),
        impl_->context); ch) impl_->rm_armors_ch = std::move(*ch);

  // 3D 面板 Marker 话题：cdr 编码 + ros2msg schema（与 ros2_foxglove_bridge 相同）
  const auto & ma_text = marker_array_schema();
  if (auto ch = foxglove::RawChannel::create(
        "/detector/marker", "cdr", cdr_schema("visualization_msgs/msg/MarkerArray", ma_text),
        impl_->context); ch) impl_->detector_marker_ch = std::move(*ch);
  if (auto ch = foxglove::RawChannel::create(
        "/tracker/marker", "cdr", cdr_schema("visualization_msgs/msg/MarkerArray", ma_text),
        impl_->context); ch) impl_->tracker_marker_ch = std::move(*ch);

  const auto & ps_text = point_stamped_schema();
  if (auto ch = foxglove::RawChannel::create(
        "/aiming_point", "cdr", cdr_schema("geometry_msgs/msg/PointStamped", ps_text),
        impl_->context); ch) impl_->aiming_point_ch = std::move(*ch);

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
  auto yaml = YAML::LoadFile(config_path);

  // Parameters 面板：把 yaml 顶层参数（含嵌套）暴露给 Foxglove
  impl_->parameters.clear();
  if (yaml.IsMap()) {
    for (const auto & kv : yaml) {
      const std::string key = kv.first.as<std::string>();
      auto value = yaml_to_param(kv.second);
      if (value.is<std::string>() || value.is<double>() || value.is<int64_t>() ||
          value.is<bool>()) {
        if (value.is<std::string>())
          impl_->parameters.emplace_back(key, value.get<std::string>());
        else if (value.is<double>())
          impl_->parameters.emplace_back(key, value.get<double>());
        else if (value.is<int64_t>())
          impl_->parameters.emplace_back(key, value.get<int64_t>());
        else
          impl_->parameters.emplace_back(key, value.get<bool>());
      } else {
        impl_->parameters.emplace_back(
          key, foxglove::ParameterType::None, std::move(value));
      }
    }
  }

  if (!impl_->camera_info_ch) return;

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
  impl_->camera_calibration = calib;
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
  // 客户端通常晚于进程启动连接，随 TF 重发内参，避免错过一次性消息。
  if (impl_->camera_info_ch && impl_->camera_calibration) {
    auto calib = *impl_->camera_calibration;
    calib.timestamp = to_timestamp(ns);
    impl_->camera_info_ch->log(calib, ns);
  }
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
  bool aim_valid, const Eigen::Vector4d & aim_xyza, const Eigen::Quaterniond & gimbal_world_q)
{
  if (!impl_->scene_ch || !has_clients()) return;

  const uint64_t ns = now_ns();
  SceneUpdate update;

  // 检测到的装甲板（识别器原始输出）
  {
    SceneEntity entity;
    entity.timestamp = to_timestamp(ns);
    entity.frame_id = "odom";
    entity.id = "detections";
    entity.frame_locked = false;

    for (const auto & armor : armors) {
      CubePrimitive cube;
      cube.pose = make_pose_ypr(
        armor.xyz_in_world.x(), armor.xyz_in_world.y(), armor.xyz_in_world.z(),
        armor.ypr_in_world[0], armor.ypr_in_world[1]);
      cube.size = armor_size(armor.type);
      cube.color = armor_color(armor.color);
      entity.cubes.push_back(cube);
    }
    update.entities.push_back(std::move(entity));
  }

  // 跟踪目标：估计的各装甲板 + 旋转中心 + 旋转圆 + 速度箭头 + 标签
  {
    SceneEntity entity;
    entity.timestamp = to_timestamp(ns);
    entity.frame_id = "odom";
    entity.id = "tracked_target";
    entity.frame_locked = false;

    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto x = target.ekf_x();
      const auto xyza_list = target.armor_xyza_list();
      const Eigen::Vector3d center = target_center(x);

      // 各装甲板位置（半透明立方体，带板面俯仰）
      for (const auto & xyza : xyza_list) {
        CubePrimitive cube;
        cube.pose = make_pose_ypr(xyza[0], xyza[1], xyza[2], xyza[3], armor_pitch(target.name));
        cube.size = armor_size(target.armor_type);
        cube.color = make_color(kTracked.r, kTracked.g, kTracked.b, 0.45);
        entity.cubes.push_back(cube);
      }

      // 旋转中心
      SpherePrimitive center_sphere;
      center_sphere.pose = make_pose(center.x(), center.y(), center.z(), 0.0);
      center_sphere.size = make_vector3(0.10, 0.10, 0.10);
      center_sphere.color = kTracked;
      entity.spheres.push_back(center_sphere);

      // 旋转圆环：r1（蓝）位于下排装甲板高度，r2（橙）位于上排高度
      const double radius = x[8];
      const double radius_2 = (xyza_list.size() >= 4) ? x[8] + x[9] : x[8];
      const double r1_z = x[4];
      const double r2_z = x[4] + x[10];
      constexpr int kSegments = 48;
      const auto add_ring = [&](double r, double z, const Color & color) {
        if (r <= 0.02) return;
        LinePrimitive ring;
        ring.type = LinePrimitive::LineType::LINE_LOOP;
        ring.thickness = 0.01;
        ring.color = color;
        for (int i = 0; i < kSegments; ++i) {
          const double a = i * 2.0 * M_PI / kSegments;
          ring.points.push_back(
            Point3{x[0] + r * std::cos(a), x[2] + r * std::sin(a), z});
        }
        entity.lines.push_back(ring);
      };
      add_ring(radius, r1_z, make_color(0.38, 0.65, 1.0, 0.6));
      if (radius_2 > radius + 0.005) add_ring(radius_2, r2_z, make_color(1.0, 0.57, 0.25, 0.6));

      // 旋转中心到各估计装甲板的辐条（展示四板相对位置）
      for (const auto & xyza : xyza_list) {
        LinePrimitive spoke;
        spoke.type = LinePrimitive::LineType::LINE_STRIP;
        spoke.thickness = 0.005;
        spoke.color = make_color(0.6, 0.65, 0.75, 0.35);
        spoke.points.push_back(Point3{center.x(), center.y(), center.z()});
        spoke.points.push_back(Point3{xyza[0], xyza[1], xyza[2]});
        entity.lines.push_back(spoke);
      }

      // 线速度箭头
      const Eigen::Vector3d vel(x[1], x[3], x[5]);
      if (vel.norm() > 0.05) {
        entity.arrows.push_back(
          make_arrow(center, center + vel, 0.03, make_color(1.0, 0.9, 0.1, 1.0)));
      }

      // 角速度箭头（沿 +z，长度随 |v_yaw| 变化）
      const double v_yaw = x[7];
      if (std::abs(v_yaw) > 0.2) {
        const double len = std::min(1.0, std::abs(v_yaw) * 0.1);
        const Eigen::Vector3d tip = center + Eigen::Vector3d(0.0, 0.0, std::copysign(len, v_yaw));
        entity.arrows.push_back(
          make_arrow(center, tip, 0.02, make_color(0.2, 1.0, 1.0, 1.0)));
      }

      // 目标标签
      TextPrimitive text;
      text.pose = make_pose(center.x(), center.y(), center.z() + 0.45, 0.0);
      text.billboard = true;
      text.scale_invariant = true;
      text.font_size = 14.0;
      text.color = kTracked;
      text.text = fmt::format(
        "{} | v_yaw={:+.1f}rad/s\nr1={:.3f}m r2={:.3f}m",
        auto_aim::ARMOR_NAMES.at(static_cast<size_t>(target.name)), v_yaw, radius, radius_2);
      entity.texts.push_back(std::move(text));
    }
    update.entities.push_back(std::move(entity));
  }

  // 自车云台：光轴 / 相机朝向（"self"）
  {
    SceneEntity entity;
    entity.timestamp = to_timestamp(ns);
    entity.frame_id = "odom";
    entity.id = "self";
    entity.frame_locked = false;

    // 云台 +x 指向（odom 系）
    const Eigen::Vector3d gimbal_dir = gimbal_world_q * Eigen::Vector3d::UnitX();
    entity.arrows.push_back(
      make_arrow(Eigen::Vector3d::Zero(), gimbal_dir * 1.5, 0.02, make_color(0.2, 0.9, 1.0, 0.9)));

    // 相机光轴（相机坐标系 +z）
    const Eigen::Vector3d camera_dir =
      gimbal_world_q * (impl_->camera_rotation * Eigen::Vector3d::UnitZ());
    entity.arrows.push_back(make_arrow(
      impl_->camera_translation, impl_->camera_translation + camera_dir * 1.5, 0.015,
      make_color(1.0, 0.3, 1.0, 0.7)));

    SpherePrimitive origin;
    origin.pose = make_pose(0.0, 0.0, 0.0, 0.0);
    origin.size = make_vector3(0.08, 0.08, 0.08);
    origin.color = make_color(0.2, 0.9, 1.0, 1.0);
    entity.spheres.push_back(origin);

    update.entities.push_back(std::move(entity));
  }

  // 瞄准点 + 指向线 / 箭头
  {
    SceneEntity entity;
    entity.timestamp = to_timestamp(ns);
    entity.frame_id = "odom";
    entity.id = "aiming_point";
    entity.frame_locked = false;

    if (aim_valid) {
      SpherePrimitive point;
      point.pose = make_pose(aim_xyza[0], aim_xyza[1], aim_xyza[2], aim_xyza[3]);
      point.size = make_vector3(0.07, 0.07, 0.07);
      point.color = kAim;
      entity.spheres.push_back(point);

      const Eigen::Vector3d aim(aim_xyza[0], aim_xyza[1], aim_xyza[2]);
      entity.arrows.push_back(
        make_arrow(Eigen::Vector3d::Zero(), aim, 0.012, make_color(kAim.r, kAim.g, kAim.b, 0.8)));

      TextPrimitive text;
      text.pose = make_pose(aim.x(), aim.y(), aim.z() + 0.15, 0.0);
      text.billboard = true;
      text.scale_invariant = true;
      text.font_size = 12.0;
      text.color = kAim;
      text.text = "aim";
      entity.texts.push_back(std::move(text));
    }
    update.entities.push_back(std::move(entity));
  }

  impl_->scene_ch->log(update, ns);
}

void FoxgloveServer::publish_prediction(
  const std::list<auto_aim::Target> & targets, double horizon_s, double step_s)
{
  if (!impl_->prediction_ch || !has_clients()) return;
  if (step_s <= 0.0) step_s = 0.02;

  const uint64_t ns = now_ns();
  SceneUpdate update;

  SceneEntity entity;
  entity.timestamp = to_timestamp(ns);
  entity.frame_id = "odom";
  entity.id = "target_prediction";
  entity.frame_locked = false;

  const int steps = std::max(1, static_cast<int>(horizon_s / step_s));

  for (const auto & target : targets) {
    // 复制一份目标，向前预测，不影响主流程的估计器状态
    auto predictor = target;

    // 最近装甲板轨迹（瞄准器实际跟随的轨迹，小陀螺时会跳变）
    LinePrimitive armor_path;
    armor_path.type = LinePrimitive::LineType::LINE_STRIP;
    armor_path.thickness = 0.008;
    armor_path.color = make_color(1.0, 0.85, 0.1, 0.9);
    // 旋转中心轨迹（平移量，平滑）
    LinePrimitive center_path;
    center_path.type = LinePrimitive::LineType::LINE_STRIP;
    center_path.thickness = 0.006;
    center_path.color = make_color(1.0, 0.85, 0.1, 0.4);

    const auto add_sample = [&](double frac) {
      const Eigen::Vector3d armor_xyz = nearest_armor_xyz(predictor);
      const Eigen::Vector3d center = target_center(predictor.ekf_x());
      armor_path.points.push_back(Point3{armor_xyz.x(), armor_xyz.y(), armor_xyz.z()});
      center_path.points.push_back(Point3{center.x(), center.y(), center.z()});
    };

    add_sample(0.0);
    for (int i = 1; i <= steps; ++i) {
      predictor.predict(step_s);
      add_sample(static_cast<double>(i) / steps);
    }

    entity.lines.push_back(std::move(armor_path));
    entity.lines.push_back(std::move(center_path));

    // 未来若干时刻的幽灵装甲板（预测位置）
    auto ghost = target;
    const int ghost_stride = std::max(1, steps / 3);
    for (int i = 1; i <= steps; i += ghost_stride) {
      ghost.predict(step_s * ghost_stride);
      const double frac = static_cast<double>(i) / steps;
      for (const auto & xyza : ghost.armor_xyza_list()) {
        CubePrimitive cube;
        cube.pose = make_pose(xyza[0], xyza[1], xyza[2], xyza[3]);
        cube.size = armor_size(ghost.armor_type);
        cube.color = make_color(1.0, 0.85, 0.1, 0.35 * (1.0 - frac) + 0.05);
        entity.cubes.push_back(cube);
      }
    }
  }

  update.entities.push_back(std::move(entity));
  impl_->prediction_ch->log(update, ns);
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
  // 原布局主视图：/detector/result_img/compressed（同一份 JPEG）
  if (impl_->result_img_ch) impl_->result_img_ch->log(msg, ns);
}

void FoxgloveServer::publish_number_image(const std::list<auto_aim::Armor> & armors)
{
  if (!impl_->number_img_ch || !has_clients()) return;

  const uint64_t ns = now_ns();
  // 数字图案拼接图很小（每帧 <2KB），无需限频

  // 把各装甲板的数字图案拼成一排；没有图案时放占位图
  cv::Mat mosaic;
  int count = 0;
  for (const auto & a : armors) {
    if (a.pattern.empty() || count >= 8) continue;
    cv::Mat p;
    cv::resize(a.pattern, p, cv::Size(80, 40));
    if (mosaic.empty()) mosaic = p;
    else {
      cv::Mat tmp;
      cv::hconcat(mosaic, p, tmp);
      mosaic = tmp;
    }
    ++count;
  }
  if (mosaic.empty()) mosaic = cv::Mat::zeros(40, 80, CV_8UC3);

  std::vector<uint8_t> buf;
  std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 80};
  cv::imencode(".jpg", mosaic, buf, params);

  CompressedImage msg;
  msg.timestamp = to_timestamp(ns);
  msg.frame_id = "camera_optical_frame";
  msg.format = "jpeg";
  msg.data.resize(buf.size());
  std::memcpy(msg.data.data(), buf.data(), buf.size());
  impl_->number_img_ch->log(msg, ns);
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

void FoxgloveServer::publish_rmvision(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
  const Eigen::Vector4d & aim_xyza)
{
  if (!has_clients()) return;
  const uint64_t ns = now_ns();
  nlohmann::json header = {{"stamp", {{"sec", ns / 1000000000ULL}, {"nanosec", ns % 1000000000ULL}}},
                           {"frame_id", "camera_optical_frame"}};
  if (impl_->rm_armors_ch) {
    nlohmann::json msg = {{"header", header}, {"armors", nlohmann::json::array()}};
    for (const auto & a : armors) {
      nlohmann::json item;
      item["number"] = auto_aim::ARMOR_NAMES.at(static_cast<size_t>(a.name));
      item["type"] = auto_aim::ARMOR_TYPES.at(static_cast<size_t>(a.type));
      item["distance_to_image_center"] = static_cast<float>(
        std::hypot(a.center.x - 720.0, a.center.y - 540.0));
      item["pose"] = {{"position", {{"x", a.xyz_in_gimbal.x()}, {"y", a.xyz_in_gimbal.y()}, {"z", a.xyz_in_gimbal.z()}}},
                      {"orientation", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"w", 1.0}}}};
      msg["armors"].push_back(std::move(item));
    }
    const auto data = msg.dump();
    impl_->rm_armors_ch->log(reinterpret_cast<const std::byte *>(data.data()), data.size(), ns);
  }
  nlohmann::json info_header = {{"stamp", {{"sec", ns / 1000000000ULL}, {"nanosec", ns % 1000000000ULL}}}};
  if (impl_->rm_target_ch) {
    nlohmann::json msg = {{"header", header}, {"tracking", false}, {"id", ""}, {"armors_num", 0},
      {"position", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
      {"velocity", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}}, {"yaw", 0.0}, {"v_yaw", 0.0},
      {"radius_1", 0.0}, {"radius_2", 0.0}, {"dz", 0.0}};
    if (!targets.empty()) {
      const auto & t = targets.front();
      const auto x = t.ekf_x();
      const int armor_num = static_cast<int>(t.armor_xyza_list().size());
      // 11 维 EKF：x[8]=r1，x[9]=l(r2-r1)，x[10]=dz；4 板目标 r2 = r1 + l
      const double r1 = x[8];
      const double r2 = (armor_num >= 4) ? x[8] + x[9] : x[8];
      msg["tracking"] = true;
      msg["armors_num"] = armor_num;
      msg["position"] = {{"x", x[0]}, {"y", x[2]}, {"z", x[4]}};
      msg["velocity"] = {{"x", x[1]}, {"y", x[3]}, {"z", x[5]}};
      msg["yaw"] = x[6]; msg["v_yaw"] = x[7]; msg["id"] = auto_aim::ARMOR_NAMES.at(static_cast<size_t>(t.name));
      msg["radius_1"] = r1; msg["radius_2"] = r2; msg["dz"] = x[10];
    }
    const auto data = msg.dump();
    impl_->rm_target_ch->log(reinterpret_cast<const std::byte *>(data.data()), data.size(), ns);
  }
  if (impl_->rm_info_ch) {
    // 帧间抖动指标：中心位移与 yaw 变化量（跟踪丢失时清零）
    double position_diff = 0.0, yaw_diff = 0.0;
    if (!targets.empty()) {
      const auto x = targets.front().ekf_x();
      const Eigen::Vector3d center(x[0], x[2], x[4]);
      if (impl_->has_last_target) {
        position_diff = (center - impl_->last_center).norm();
        yaw_diff = tools::limit_rad(x[6] - impl_->last_yaw);
      }
      impl_->last_center = center;
      impl_->last_yaw = x[6];
      impl_->has_last_target = true;
    } else {
      impl_->has_last_target = false;
    }
    nlohmann::json msg = {{"header", info_header}, {"position_diff", position_diff}, {"yaw_diff", yaw_diff},
      {"position", {{"x", aim_xyza[0]}, {"y", aim_xyza[1]}, {"z", aim_xyza[2]}}},
      {"yaw", aim_xyza[3]}};
    const auto data = msg.dump();
    impl_->rm_info_ch->log(reinterpret_cast<const std::byte *>(data.data()), data.size(), ns);
  }

  // /aiming_point：geometry_msgs/msg/PointStamped（odom 系瞄准点，原布局 3D 面板）
  if (impl_->aiming_point_ch) {
    CdrWriter w;
    w.i32(static_cast<int32_t>(ns / 1000000000ULL));
    w.u32(static_cast<uint32_t>(ns % 1000000000ULL));
    w.str("odom");
    w.f64(aim_xyza[0]);
    w.f64(aim_xyza[1]);
    w.f64(aim_xyza[2]);
    log_raw(impl_->aiming_point_ch, w.buf(), ns);
  }
}

// 绕 Z 轴 yaw 的四元数（odom 系里装甲板/目标的朝向）
Quaternion yaw_quaternion(double yaw)
{
  return make_quaternion(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
}

void FoxgloveServer::publish_detector_markers(const std::list<auto_aim::Armor> & armors)
{
  if (!impl_->detector_marker_ch || !has_clients()) return;

  const uint64_t ns = now_ns();
  const uint32_t sec = static_cast<uint32_t>(ns / 1000000000ULL);
  const uint32_t nsec = static_cast<uint32_t>(ns % 1000000000ULL);

  // 对齐 rm_auto_aim：每个装甲板一个 CUBE + 一个 TEXT_VIEW_FACING
  std::vector<RosMarker> markers;
  int id = 0;
  for (const auto & a : armors) {
    const double width = (a.type == auto_aim::big) ? 0.230 : 0.135;
    const auto pos = a.xyz_in_world;
    const double yaw = a.ypr_in_world[0];

    RosMarker cube;
    cube.ns = "armors";
    cube.id = ++id;
    cube.type = 1;  // CUBE
    cube.action = 0;
    cube.position = pos;
    cube.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    cube.scale = {0.05, width, 0.125};
    cube.color = {0.0f, 0.5f, 1.0f, 1.0f};
    cube.lifetime_ns = 100000000;  // 0.1s
    markers.push_back(std::move(cube));

    RosMarker text;
    text.ns = "classification";
    text.id = id;
    text.type = 9;  // TEXT_VIEW_FACING
    text.action = 0;
    text.position = pos + Eigen::Vector3d(0.0, -0.1, 0.0);
    text.scale = {0.0, 0.0, 0.1};
    text.color = {1.0f, 1.0f, 1.0f, 1.0f};
    text.lifetime_ns = 100000000;
    text.text = auto_aim::ARMOR_NAMES.at(static_cast<size_t>(a.name));
    markers.push_back(std::move(text));
  }

  CdrWriter w;
  w.seq_begin(static_cast<uint32_t>(markers.size()), 4);
  for (const auto & m : markers) write_marker(w, m, sec, nsec, "odom");
  log_raw(impl_->detector_marker_ch, w.buf(), ns);
}

void FoxgloveServer::publish_tracker_markers(const std::list<auto_aim::Target> & targets)
{
  if (!impl_->tracker_marker_ch || !has_clients()) return;

  const uint64_t ns = now_ns();
  const uint32_t sec = static_cast<uint32_t>(ns / 1000000000ULL);
  const uint32_t nsec = static_cast<uint32_t>(ns % 1000000000ULL);

  // 对齐 rm_auto_aim：SPHERE 位置 + 线速度/角速度 ARROW + 各装甲板 CUBE
  std::vector<RosMarker> markers;
  if (!targets.empty()) {
    const auto & t = targets.front();
    const auto x = t.ekf_x();
    const double xc = x[0], yc = x[2], za = x[4];
    const double vx = x[1], vy = x[3], vz = x[5];
    const double v_yaw = x[7];
    const double dz = x[10];
    const Eigen::Vector3d center(xc, yc, za + dz / 2.0);

    {
      RosMarker position;
      position.ns = "position";
      position.id = 0;
      position.type = 2;  // SPHERE
      position.action = 0;
      position.position = center;
      position.scale = {0.1, 0.1, 0.1};
      position.color = {0.0f, 1.0f, 0.0f, 1.0f};
      markers.push_back(std::move(position));
    }
    {
      RosMarker arrow;
      arrow.ns = "linear_v";
      arrow.id = 0;
      arrow.type = 0;  // ARROW
      arrow.action = 0;
      arrow.scale = {0.03, 0.05, 0.1};
      arrow.color = {1.0f, 1.0f, 0.0f, 1.0f};
      arrow.points = {center, center + Eigen::Vector3d(vx, vy, vz)};
      markers.push_back(std::move(arrow));
    }
    {
      RosMarker arrow;
      arrow.ns = "angular_v";
      arrow.id = 0;
      arrow.type = 0;  // ARROW
      arrow.action = 0;
      arrow.scale = {0.03, 0.05, 0.1};
      arrow.color = {0.0f, 1.0f, 1.0f, 1.0f};
      arrow.points = {center, center + Eigen::Vector3d(0.0, 0.0, v_yaw / M_PI)};
      markers.push_back(std::move(arrow));
    }

    const double width = (t.armor_type == auto_aim::big) ? 0.230 : 0.135;
    const auto xyza_list = t.armor_xyza_list();
    for (size_t i = 0; i < xyza_list.size(); ++i) {
      RosMarker cube;
      cube.ns = "armors";
      cube.id = static_cast<int32_t>(i);
      cube.type = 1;  // CUBE
      cube.action = 0;
      cube.position = xyza_list[i].head<3>();
      cube.orientation =
        Eigen::Quaterniond(
          Eigen::AngleAxisd(xyza_list[i][3], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(armor_pitch(t.name), Eigen::Vector3d::UnitY()));
      cube.scale = {0.03, width, 0.125};
      cube.color = {1.0f, 0.0f, 0.0f, 1.0f};
      markers.push_back(std::move(cube));
    }
  }

  CdrWriter w;
  w.seq_begin(static_cast<uint32_t>(markers.size()), 4);
  for (const auto & m : markers) write_marker(w, m, sec, nsec, "odom");
  log_raw(impl_->tracker_marker_ch, w.buf(), ns);
}

void FoxgloveServer::publish_debug(
  double fps, const io::Command & command, const std::list<auto_aim::Target> & targets,
  const std::list<auto_aim::Armor> & armors, bool aim_valid, const Eigen::Vector4d & aim_xyza,
  const Eigen::Vector3d & gimbal_ypr)
{
  if (!impl_->debug_ch || !has_clients()) return;

  // 曲线字段，Plot 面板可直接选
  nlohmann::json dbg;
  dbg["fps"] = fps;
  dbg["yaw"] = command.yaw;
  dbg["pitch"] = command.pitch;
  dbg["gimbal_yaw"] = gimbal_ypr[0];
  dbg["gimbal_pitch"] = gimbal_ypr[1];
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

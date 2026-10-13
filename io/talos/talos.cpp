#include "talos.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
constexpr std::uint32_t kShmMagic = 0x54414C05;
constexpr std::uint32_t kShmVersion = 3;
constexpr std::uint8_t kFlagNew = 0x80;
constexpr std::uint8_t kIndexMask = 0x03;
constexpr std::size_t kMetaSize = 4160;
constexpr std::size_t kHitEventCapacity = 16;
constexpr std::size_t kImageWidth = 1440;
constexpr std::size_t kImageHeight = 1080;
constexpr std::size_t kImageSize = kImageWidth * kImageHeight * 3;
constexpr std::size_t kImagePoolSize = kImageSize * 3;
constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;

struct alignas(32) ImageMeta
{
  std::uint64_t seq;
  std::uint64_t timestamp_ns;
  std::uint32_t width;
  std::uint32_t height;
  std::uint8_t buffer_id;
  std::uint8_t format;
  std::uint8_t padding[6];
};

struct alignas(64) PoseMeta
{
  std::uint64_t frame_seq;
  float position[3];
  float quaternion[4];
  std::uint64_t timestamp_ns;
  std::uint8_t padding[16];
};

template <typename Slot>
struct alignas(64) TripleBuffer
{
  std::uint8_t state;
  std::uint8_t write_idx;
  std::uint8_t read_idx;
  std::uint8_t padding[61];
  Slot slots[3];
};

struct alignas(64) ShmHeader
{
  std::uint32_t magic;
  std::uint32_t version;
  std::uint64_t created_ns;
  std::uint64_t heartbeat_ns;
  std::uint32_t image_width;
  std::uint32_t image_height;
  std::uint8_t padding[32];
};

struct alignas(32) GimbalCmd
{
  std::uint64_t timestamp_ns;
  float yaw_deg;
  float pitch_deg;
  float distance_m;
  std::uint8_t fire_advice;
  std::uint8_t padding[11];
};

struct HitEventWire
{
  std::uint64_t seq;
  std::uint64_t timestamp_ns;
  std::uint8_t team;
  std::uint8_t label;
  std::uint8_t rune;
  std::uint8_t padding[5];
};
static_assert(sizeof(HitEventWire) == 24);

struct alignas(64) HitEventRingWire
{
  std::uint64_t write_seq;
  std::uint32_t launch_count;
  std::uint32_t accurate_count;
  HitEventWire events[16];
};
static_assert(sizeof(HitEventRingWire) == 448);

struct ShmMetaRegion
{
  ShmHeader header;
  TripleBuffer<ImageMeta> image;
  TripleBuffer<PoseMeta> poses[5];
  TripleBuffer<GimbalCmd> gimbal_cmd;
  std::uint8_t camera_info[128];
  std::uint8_t chassis_observation[128];
  std::uint8_t ground_truth[1664];
  std::uint8_t runtime_state[64];
  HitEventRingWire hit_events;
};

static_assert(sizeof(ImageMeta) == 32);
static_assert(sizeof(PoseMeta) == 64);
static_assert(sizeof(TripleBuffer<ImageMeta>) == 192);
static_assert(sizeof(TripleBuffer<PoseMeta>) == 256);
static_assert(sizeof(GimbalCmd) == 32);
static_assert(sizeof(ShmHeader) == 64);
static_assert(offsetof(ShmMetaRegion, image) == 64);
static_assert(offsetof(ShmMetaRegion, poses) == 256);
static_assert(offsetof(ShmMetaRegion, gimbal_cmd) == 1536);
static_assert(offsetof(ShmMetaRegion, camera_info) == 1728);
static_assert(offsetof(ShmMetaRegion, chassis_observation) == 1856);
static_assert(offsetof(ShmMetaRegion, ground_truth) == 1984);
static_assert(offsetof(ShmMetaRegion, runtime_state) == 3648);
static_assert(offsetof(ShmMetaRegion, hit_events) == 3712);
static_assert(sizeof(ShmMetaRegion) == kMetaSize);

// Wire layout for GroundTruthBatch (mirrors Rust crates/talos-ipc/src/layout.rs).
struct alignas(32) GroundTruthTargetWire
{
  std::uint64_t frame_seq;
  std::uint64_t timestamp_ns;
  std::uint8_t team;
  std::uint8_t armor_label;
  std::uint8_t is_outpost;
  std::uint8_t padding1;
  float position[3];
  float vyaw;
  float yaw;
  std::uint8_t padding[24];
};
static_assert(sizeof(GroundTruthTargetWire) == 64);

struct alignas(64) GroundTruthRuneWire
{
  std::uint8_t bytes[128];
};
static_assert(sizeof(GroundTruthRuneWire) == 128);

struct alignas(64) GroundTruthBatchWire
{
  std::uint64_t frame_seq;
  std::uint64_t timestamp_ns;
  std::uint32_t target_count;
  std::uint32_t rune_count;
  GroundTruthTargetWire targets[16];
  GroundTruthRuneWire runes[4];
  std::uint8_t padding[64];
};
static_assert(sizeof(GroundTruthBatchWire) == 1664);

std::uint8_t load_state(const std::uint8_t * state)
{
  return __atomic_load_n(state, __ATOMIC_ACQUIRE);
}

bool consume_slot(
  std::uint8_t * state, std::uint8_t & read_idx, std::uint8_t & slot_idx)
{
  auto expected = load_state(state);
  if ((expected & kFlagNew) == 0) return false;

  const auto ready_idx = static_cast<std::uint8_t>(expected & kIndexMask);
  const auto desired = read_idx;
  if (!__atomic_compare_exchange_n(
        state, &expected, desired, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
    return false;
  }

  read_idx = ready_idx;
  slot_idx = ready_idx;
  return true;
}

std::uint64_t now_unix_ns()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

std::runtime_error io_error(const char * operation, const char * path)
{
  return std::runtime_error(
    std::string(operation) + " " + path + ": " + std::strerror(errno));
}

class Mapping
{
public:
  Mapping(const char * path, std::size_t size) : path_(path), size_(size)
  {
    fd_ = ::open(path, O_RDWR);
    if (fd_ < 0) throw io_error("open", path);

    struct stat info {
    };
    if (::fstat(fd_, &info) != 0 || static_cast<std::size_t>(info.st_size) < size_) {
      const auto error = io_error("stat", path);
      ::close(fd_);
      fd_ = -1;
      throw error;
    }

    data_ = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (data_ == MAP_FAILED) {
      const auto error = io_error("mmap", path);
      ::close(fd_);
      fd_ = -1;
      throw error;
    }
  }

  ~Mapping()
  {
    if (data_ != MAP_FAILED) ::munmap(data_, size_);
    if (fd_ >= 0) ::close(fd_);
  }

  Mapping(const Mapping &) = delete;
  Mapping & operator=(const Mapping &) = delete;

  template <typename T>
  T * as()
  {
    return static_cast<T *>(data_);
  }

private:
  const char * path_;
  std::size_t size_;
  int fd_ = -1;
  void * data_ = MAP_FAILED;
};
}

namespace io
{
struct Talos::Impl
{
  Mapping meta_mapping{"/tmp/talos_ipc_meta", kMetaSize};
  Mapping image_mapping{"/tmp/talos_ipc_image_pool", kImagePoolSize};
  ShmMetaRegion * meta = meta_mapping.as<ShmMetaRegion>();
  std::uint8_t image_read_idx = 2;
  std::array<std::uint8_t, 4> pose_read_idx{2, 2, 2, 2};
  std::uint8_t command_write_idx = 0;
  Eigen::Quaterniond gimbal_quaternion = Eigen::Quaterniond::Identity();
  std::chrono::steady_clock::time_point gimbal_timestamp{};
  std::chrono::steady_clock::duration clock_offset{};
  bool clock_offset_initialized = false;
  std::uint64_t last_image_seq = 0;
  Poses poses_{};
  std::uint64_t last_image_timestamp_ns_ = 0;
  bool have_poses_ = false;
  mutable std::uint64_t last_hit_seq_ = 0;

  Impl()
  {
    if (meta->header.magic != kShmMagic || meta->header.version != kShmVersion) {
      throw std::runtime_error("invalid Talos shared-memory header");
    }
    if (meta->header.image_width != kImageWidth || meta->header.image_height != kImageHeight) {
      throw std::runtime_error("unsupported Talos image dimensions");
    }
    command_write_idx = meta->gimbal_cmd.write_idx & kIndexMask;
  }

  std::chrono::steady_clock::time_point to_steady(std::uint64_t timestamp_ns)
  {
    const auto system_timestamp = std::chrono::system_clock::time_point{
      std::chrono::nanoseconds(timestamp_ns)};
    if (!clock_offset_initialized) {
      clock_offset = std::chrono::steady_clock::now().time_since_epoch() -
                     system_timestamp.time_since_epoch();
      clock_offset_initialized = true;
    }
    return std::chrono::steady_clock::time_point{system_timestamp.time_since_epoch() + clock_offset};
  }

  bool consume_frame(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp)
  {
    std::uint8_t image_slot = 0;
    if (!consume_slot(&meta->image.state, image_read_idx, image_slot)) return false;

    ImageMeta image_meta = meta->image.slots[image_slot];
    std::array<PoseMeta, 4> poses{};
    for (std::size_t index = 0; index < poses.size(); ++index) {
      std::uint8_t pose_slot = 0;
      if (!consume_slot(
            &meta->poses[index].state, pose_read_idx[index], pose_slot)) {
        return false;
      }
      poses[index] = meta->poses[index].slots[pose_slot];
    }

    if (image_meta.seq <= last_image_seq || image_meta.buffer_id >= 3 ||
        image_meta.width != kImageWidth || image_meta.height != kImageHeight) {
      return false;
    }

    const auto * source = static_cast<const std::uint8_t *>(image_mapping.as<void>()) +
                          image_meta.buffer_id * kImageSize;
    cv::Mat rgb(static_cast<int>(kImageHeight), static_cast<int>(kImageWidth), CV_8UC3);
    std::memcpy(rgb.data, source, kImageSize);
    cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);

    const auto frame_time = to_steady(image_meta.timestamp_ns);
    const auto & gimbal = poses[0];
    gimbal_quaternion = Eigen::Quaterniond(
      gimbal.quaternion[0], gimbal.quaternion[1], gimbal.quaternion[2],
      gimbal.quaternion[3]);
    if (gimbal_quaternion.norm() > 0.0) gimbal_quaternion.normalize();
    gimbal_timestamp = frame_time;
    timestamp = frame_time;
    last_image_seq = image_meta.seq;

    // Cache all four poses and the raw image timestamp as owned snapshots.
    // PoseIndex order: 0=Gimbal, 1=Odom, 2=Muzzle, 3=Camera.
    auto to_pose = [](const PoseMeta & p) {
      Pose out;
      out.position = Eigen::Vector3d(p.position[0], p.position[1], p.position[2]);
      Eigen::Quaterniond q(
        p.quaternion[0], p.quaternion[1], p.quaternion[2], p.quaternion[3]);
      if (q.norm() > 0.0) q.normalize();
      out.orientation = q;
      return out;
    };
    poses_.gimbal = to_pose(poses[0]);
    poses_.odom = to_pose(poses[1]);
    poses_.muzzle = to_pose(poses[2]);
    poses_.camera = to_pose(poses[3]);
    have_poses_ = true;
    last_image_timestamp_ns_ = image_meta.timestamp_ns;
    return true;
  }

  void publish_command(
    bool control, bool fire, float yaw, float pitch, float distance_m)
  {
    auto & buffer = meta->gimbal_cmd;
    auto & command = buffer.slots[command_write_idx];
    command.timestamp_ns = now_unix_ns();
    command.yaw_deg = yaw * static_cast<float>(kDegreesPerRadian);
    // 俯仰约定与仿真器消费端对齐（src/talos/plugin.rs:144）：
    // 消费端会再取一次负，交给 GimbalAimTarget::from_solver_degrees，
    // 该函数要求入参为“自竖直起算角”（90°=水平）。
    // command.pitch 为自水平起算角（抬头为负），故此处发送 -(pitch度 + 90)，
    // 使消费端取负后得到 90 + command.pitch。注意：仅 pitch 需要，yaw 直接透传。
    command.pitch_deg = -(pitch * static_cast<float>(kDegreesPerRadian) + 90.0F);
    command.distance_m = control ? distance_m : -1.0F;
    command.fire_advice = static_cast<std::uint8_t>(fire ? 1 : 0);

    const auto old = __atomic_exchange_n(
      &buffer.state, static_cast<std::uint8_t>(command_write_idx | kFlagNew),
      __ATOMIC_ACQ_REL);
    command_write_idx = old & kIndexMask;
  }
};

Talos::Talos() : impl_(std::make_unique<Impl>()) {}

Talos::~Talos() = default;

bool Talos::read_image(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp)
{
  return impl_->consume_frame(image, timestamp);
}

Eigen::Quaterniond Talos::get_gimbal_quaternion(
  std::chrono::steady_clock::time_point timestamp) const
{
  (void)timestamp;
  return impl_->gimbal_quaternion;
}

void Talos::send_gimbal_command(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
  float pitch_vel, float pitch_acc, float distance_m)
{
  (void)yaw_vel;
  (void)yaw_acc;
  (void)pitch_vel;
  (void)pitch_acc;
  impl_->publish_command(control, fire, yaw, pitch, distance_m);
}

bool Talos::read_poses(Poses & out) const
{
  if (!impl_->have_poses_) return false;
  out = impl_->poses_;
  return true;
}

std::vector<GroundTruthTarget> Talos::read_ground_truth() const
{
  const auto * batch =
    reinterpret_cast<const GroundTruthBatchWire *>(impl_->meta->ground_truth);

  GroundTruthBatchWire snapshot{};
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint64_t before = batch->frame_seq;
    snapshot = *batch;
    if (snapshot.frame_seq == before) break;
  }

  const std::uint32_t count = snapshot.target_count < 16 ? snapshot.target_count : 16;
  std::vector<GroundTruthTarget> out;
  out.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    const auto & t = snapshot.targets[i];
    GroundTruthTarget target;
    target.position = Eigen::Vector3d(t.position[0], t.position[1], t.position[2]);
    target.yaw = t.yaw;
    target.vyaw = t.vyaw;
    target.team = t.team;
    target.armor_label = t.armor_label;
    out.push_back(target);
  }
  return out;
}

std::uint64_t Talos::last_image_timestamp_ns() const
{
  return impl_->last_image_timestamp_ns_;
}

std::vector<HitEvent> Talos::read_hit_events() const
{
  std::vector<HitEvent> out;
  const auto & ring = impl_->meta->hit_events;
  const std::uint64_t write = ring.write_seq;
  std::uint64_t start = impl_->last_hit_seq_;
  if (write <= start) return out;
  if (write - start > kHitEventCapacity) start = write - kHitEventCapacity;

  for (std::uint64_t seq = start; seq < write; ++seq) {
    const auto & wire = ring.events[seq % kHitEventCapacity];
    if (wire.seq != seq) continue;
    HitEvent event;
    event.timestamp_ns = wire.timestamp_ns;
    event.team = wire.team;
    event.label = wire.label;
    event.rune = wire.rune;
    out.push_back(event);
  }
  impl_->last_hit_seq_ = write;
  return out;
}

std::uint32_t Talos::launch_count() const
{
  return impl_->meta->hit_events.launch_count;
}
}

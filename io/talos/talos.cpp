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
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
constexpr std::uint32_t kShmMagic = 0x54414C05;
constexpr std::uint32_t kShmVersion = 2;
constexpr std::uint8_t kFlagNew = 0x80;
constexpr std::uint8_t kIndexMask = 0x03;
constexpr std::size_t kMetaSize = 3712;
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
static_assert(sizeof(ShmMetaRegion) == kMetaSize);

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
}

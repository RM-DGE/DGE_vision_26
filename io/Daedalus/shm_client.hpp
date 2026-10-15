#ifndef IO__DAEDALUS__SHM_CLIENT_HPP
#define IO__DAEDALUS__SHM_CLIENT_HPP

// Daedalus 仿真器共享内存客户端（C++17 精简版）
// 协议契约与 bevy_robomaster_simulator/crates/talos-ipc/src/layout.rs (SHM_VERSION=2) 一致。
// 原始参考实现: talos26/crates/hardware_daedalus/src/shm_client.hpp (C++23 std::expected 版)

#include "shm_layout.hpp"
#include "shm_triple_buffer.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <optional>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace ipc
{

// 与 Rust 侧 static_assert 对齐的布局常量
static_assert(sizeof(ImageTripleBuffer) == 192);
static_assert(sizeof(PoseTripleBuffer) == 256);
static_assert(sizeof(GimbalTripleBuffer) == 192);
static_assert(sizeof(CameraInfo) == 128);
static_assert(sizeof(ShmHeader) == 64);
static_assert(sizeof(ImageMeta) == 32);
static_assert(sizeof(PoseMeta) == 64);
static_assert(sizeof(GimbalCmd) == 32);
static_assert(sizeof(ShmMetaRegion) == 3712);

class ShmClient
{
public:
  struct ImageFrame
  {
    cv::Mat image;         // 零拷贝 cv::Mat（指向共享内存，下次 recv 前有效）
    uint64_t seq;
    uint64_t timestamp_ns;
  };

  struct Pose
  {
    double x, y, z;
    double qw, qx, qy, qz;
    uint64_t frame_seq;
    uint64_t timestamp_ns;
  };

  ShmClient()
  {
    meta_fd_ = open_file("/tmp/" + std::string(SHM_NAME_META));
    map_region(meta_fd_, sizeof(ShmMetaRegion), meta_);

    pool_fd_ = open_file("/tmp/" + std::string(SHM_NAME_IMAGE_POOL));
    map_region(pool_fd_, IMAGE_POOL_SIZE, pool_);

    auto * meta = reinterpret_cast<ShmMetaRegion *>(meta_);
    if (meta->header.magic != SHM_MAGIC)
      throw std::runtime_error("daedalus shm magic mismatch (版本不一致?)");
    if (meta->header.version != SHM_VERSION)
      throw std::runtime_error("daedalus shm version mismatch (版本不一致?)");
  }

  ~ShmClient()
  {
    munmap(meta_, sizeof(ShmMetaRegion));
    munmap(pool_, IMAGE_POOL_SIZE);
    close(meta_fd_);
    close(pool_fd_);
  }

  const CameraInfo & camera_info() const
  {
    return reinterpret_cast<const ShmMetaRegion *>(meta_)->camera_info;
  }

  // 读取最新真值批次（仿真器每帧直写普通内存，无三缓冲握手，直接拷贝）
  GroundTruthBatch ground_truth() const
  {
    return reinterpret_cast<const ShmMetaRegion *>(meta_)->ground_truth;
  }

  bool is_producer_alive(const uint64_t timeout_ns = 1'000'000'000) const
  {
    auto * meta = reinterpret_cast<const ShmMetaRegion *>(meta_);
    return now_ns() - meta->header.heartbeat_ns < timeout_ns;
  }

  // 尝试取最新图像帧（消费后释放 FLAG_NEW，发布端才能发下一帧）
  std::optional<ImageFrame> recv_image()
  {
    auto * meta = reinterpret_cast<ShmMetaRegion *>(meta_);
    ImageOps ops(&meta->image);
    const auto slot = ops.borrow();
    if (!slot) return std::nullopt;

    const auto & img_meta = **slot;
    uint8_t * img_data = reinterpret_cast<uint8_t *>(pool_) + img_meta.buffer_id * IMAGE_SIZE;

    int cv_type = CV_8UC3;  // 默认 RGB8 (format 0)
    if (img_meta.format == 1) cv_type = CV_8UC3;         // BGR8
    else if (img_meta.format == 2) cv_type = CV_8UC1;    // GRAY8

    ImageFrame frame;
    frame.image = cv::Mat(img_meta.height, img_meta.width, cv_type);
    std::memcpy(frame.image.data, img_data, IMAGE_SIZE);
    frame.seq = img_meta.seq;
    frame.timestamp_ns = img_meta.timestamp_ns;
    return frame;
  }

  // 取指定位姿通道的最新数据（不阻塞）
  std::optional<Pose> recv_pose(const PoseIndex index)
  {
    auto * meta = reinterpret_cast<ShmMetaRegion *>(meta_);
    PoseOps ops(&meta->poses[static_cast<int>(index)]);
    const auto slot = ops.borrow();
    if (!slot) return std::nullopt;

    const auto & p = **slot;
    Pose pose;
    pose.x = p.position[0];
    pose.y = p.position[1];
    pose.z = p.position[2];
    pose.qw = p.quaternion[0];
    pose.qx = p.quaternion[1];
    pose.qy = p.quaternion[2];
    pose.qz = p.quaternion[3];
    pose.frame_seq = p.frame_seq;
    pose.timestamp_ns = p.timestamp_ns;
    return pose;
  }

  // 发送云台命令：绝对角度制（度），fire_advice 为开火建议
  void send_gimbal_cmd(
    const float yaw_deg, const float pitch_deg, const float distance_m,
    const bool fire_advice)
  {
    auto * meta = reinterpret_cast<ShmMetaRegion *>(meta_);
    GimbalOps ops(&meta->gimbal_cmd);
    auto & cmd = ops.borrow_mut();

    cmd.timestamp_ns = now_ns();
    cmd.yaw_deg = yaw_deg;
    cmd.pitch_deg = pitch_deg;
    cmd.distance_m = distance_m;
    cmd.fire_advice = fire_advice ? 1 : 0;

    ops.publish();
  }

private:
  // 仿真器的心跳用 UNIX_EPOCH 纳秒（SystemTime），必须用同一时钟域比较
  static uint64_t now_ns()
  {
    return static_cast<uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count());
  }

  static int open_file(const std::string & path)
  {
    int fd = open(path.c_str(), O_RDWR);
    if (fd < 0) throw std::runtime_error("无法打开共享内存 " + path + "（仿真器没在运行?）");
    return fd;
  }

  static void map_region(int fd, size_t size, void * & out)
  {
    void * p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) throw std::runtime_error("mmap 失败");
    out = p;
  }

  int meta_fd_ = -1;
  int pool_fd_ = -1;
  void * meta_ = nullptr;
  void * pool_ = nullptr;
};

}  // namespace ipc

#endif  // IO__DAEDALUS__SHM_CLIENT_HPP

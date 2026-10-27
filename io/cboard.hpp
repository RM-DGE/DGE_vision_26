#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <deque>   // Added for queue implementation
#include <mutex>   // Added for thread safety

#include "io/command.hpp"
#include "io/socketserial.hpp" 
#include "tools/logger.hpp"

namespace io
{
enum Mode
{
  idle,//空闲
  auto_aim,
  small_buff,
  big_buff,
  outpost    //前哨站
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

//没有双射单位
// enum ShootMode
// {
//   left_shoot,
//   right_shoot,
//   both_shoot
// };
// const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

#pragma pack(push, 1)
struct IMUDataFrame {
    uint8_t header;
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t tail;
    // int16_t w;
};

struct StatusDataFrame {
    uint8_t header;
    int16_t bullet_speed;
    int16_t mode;
    int16_t ShootMode;
    int16_t ft_angle;
};

struct ControlDataFrame {
    uint8_t header;
    uint8_t control;//控制
    uint8_t shoot;//射击
    int16_t yaw;
    int16_t pitch;
    uint8_t tail;
    // int16_t yaw_vel;        // x1000 (单位 rad/s)
    // int16_t pitch_vel;
    // int16_t horizon_distance;//无人机水平距离
};
#pragma pack(pop)

// 串口/共享内存收到的一帧 IMU。timestamp 是接收时刻的 steady_clock，
// ROS 桥接层用它反推数据年龄，不能直接当成 ROS 时间使用。
struct ImuSample
{
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  std::chrono::steady_clock::time_point timestamp{};
};

// 下位机状态帧（弹速、模式、FT 角度等）。
struct StatusSample
{
  double bullet_speed = 0;
  int mode = 0;
  int shoot_mode = 0;
  double ft_angle = 0;
  std::chrono::steady_clock::time_point timestamp{};
};

using ImuObserver = std::function<void(const ImuSample &)>;
using StatusObserver = std::function<void(const StatusSample &)>;

// 云台通信的抽象基类：真机用 CBoard（串口），仿真用 DaedalusCBoard（共享内存）。
class CBoardBase
{
public:
  CBoardBase() = default;
  virtual ~CBoardBase() = default;

  double bullet_speed = 0;
  Mode mode = Mode::idle;
  int shoot_mode = 0;
  double ft_angle = 0;

  virtual Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) = 0;
  virtual void send(Command command) const = 0;

  // ROS 桥接层用：在串口读线程里收到完整帧后回调。
  // 回调必须只做“拷贝到 mailbox”这类短操作，禁止在回调里直接 publish。
  virtual void set_imu_observer(ImuObserver observer) { (void)observer; }
  virtual void set_status_observer(StatusObserver observer) { (void)observer; }
};

class CBoard : public CBoardBase
{
public:
  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) override;

  void send(Command command) const override;

  void set_imu_observer(ImuObserver observer) override;
  void set_status_observer(StatusObserver observer) override;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  std::deque<IMUData> queue_;
  mutable std::mutex queue_mutex_;
  const size_t queue_capacity_ = 5000;

  mutable std::mutex observer_mutex_;
  ImuObserver imu_observer_;
  StatusObserver status_observer_;

  mutable SerialPortMultithread serial_;
  IMUData data_ahead_;
  IMUData data_behind_;

  void callback(const std::vector<uint8_t> & frame);

  std::string read_yaml(const std::string & config_path);
  int read_baudrate(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP
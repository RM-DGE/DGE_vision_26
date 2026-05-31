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

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  // ShootMode shoot_mode;
  double ft_angle;

  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  
  std::deque<IMUData> queue_;
  mutable std::mutex queue_mutex_;
  const size_t queue_capacity_ = 5000;

  mutable SerialPortMultithread serial_;
  IMUData data_ahead_;
  IMUData data_behind_;

  void callback(const std::vector<uint8_t> & frame);

  std::string read_yaml(const std::string & config_path);
  int read_baudrate(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP
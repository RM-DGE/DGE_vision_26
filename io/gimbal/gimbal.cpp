/* 接收包 (Gimbal -> Vision)
结构体: GimbalToVision 总长度: 43 字节 (Byte)
Head,           uint8_t[2], 2,"固定为 'S', 'P' (0x53, 0x50)"
Mode,           uint8_t,    1,  0: 空闲1: 自瞄(不使用2: 小符3: 大符)
Q (Quaternion), float[4],   16,"归一化四元数，顺序为 w, x, y, z。"
Yaw,            float,      4,当前 Yaw 轴角度
Yaw Vel,        float,      4,当前 Yaw 轴角速度
Pitch,          float,      4,当前 Pitch 轴角度
Pitch Vel,      float,      4,当前 Pitch 轴角速度
Bullet Speed,   float,      4,当前裁判系统读取的弹速 (m/s)
Bullet Count,   uint16_t,   2,累计发射子弹数（用于判断是否发弹成功）
CRC16,          uint16_t,   2,校验码 */

/* 发送包 (Vision -> Gimbal)
结构体: VisionToGimbal 总长度: 29 字节 (Byte)
Head,           uint8_t[2], 2,"固定为 'S', 'P' (0x53, 0x50)"
Mode,           uint8_t,    1,0: 不控制1: 控制云台不开火 (瞄准)2: 控制云台且开火
Yaw,            float,      4,目标 Yaw 轴绝对角度
Yaw Vel,        float,      4,目标 Yaw 轴前馈速度
Yaw Acc,        float,      4,目标 Yaw 轴前馈加速度
Pitch,          float,      4,目标 Pitch 轴绝对角度
Pitch Vel,      float,      4,目标 Pitch 轴前馈速度
Pitch Acc,      float,      4,目标 Pitch 轴前馈加速度
CRC16,          uint16_t,   2,校验码 (Check Sum) */
#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <cstring> // For memcpy

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  port_name_ = tools::read<std::string>(yaml, "com_port");

  try {
    open_serial();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  // Wait for first data packet similar to original behavior
  tools::logger()->info("[Gimbal] Waiting for data...");
  while (!quit_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!queue_.empty()) {
          // Mimic original logic: queue_.pop() dropped the first element
          queue_.pop_front(); 
          break;
      }
  }
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (serial_.IsOpen()) serial_.Close();
}

void Gimbal::open_serial() 
{
    if (serial_.IsOpen()) serial_.Close();
    serial_.Open(port_name_);
    // Configure Serial Port (assuming 115200 8N1 as standard, adjust if config requires)
    serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE: return "IDLE";
    case GimbalMode::AUTO_AIM: return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF: return "BIG_BUFF";
    default: return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    QueueData a, b;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (queue_.size() < 2) {
            if (!queue_.empty()) return queue_.back().q;
            return Eigen::Quaterniond::Identity();
        }
        
        // Peek first two elements
        a = queue_[0];
        b = queue_[1];
        
        // If t > b.t, 'a' is obsolete. Pop 'a' and retry.
        if (t > b.t) {
            queue_.pop_front();
            continue;
        }
    }
    
    // Interpolate between a and b
    // This covers cases: t < a.t (extrapolate/clamp start) and a.t <= t <= b.t
    
    double dt = tools::delta_time(a.t, b.t);
    if (std::abs(dt) < 1e-6) return a.q; // Avoid div by zero
    
    double t_ac = tools::delta_time(a.t, t);
    double k = t_ac / dt;
    
    return a.q.slerp(k, b.q).normalized();
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_ = VisionToGimbal; // Helper assignment if supported, else manual copy
  // Ensure head is correct if passed struct was raw
  tx_data_.head[0] = 'S'; tx_data_.head[1] = 'P';
  
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    std::vector<uint8_t> buffer(sizeof(tx_data_));
    std::memcpy(buffer.data(), &tx_data_, sizeof(tx_data_));
    serial_.Write(buffer);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

//映射发送0：空闲 1：控制 2：开火
void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    std::vector<uint8_t> buffer(sizeof(tx_data_));
    std::memcpy(buffer.data(), &tx_data_, sizeof(tx_data_));
    serial_.Write(buffer);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    LibSerial::DataBuffer read_buffer;
    // 20ms timeout for reading
    serial_.Read(read_buffer, size, 20);
    if (read_buffer.size() == size) 
    {
        std::memcpy(buffer, read_buffer.data(), size);
        return true;
    }
    return false;
  } catch (...) {
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!serial_.IsOpen()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        reconnect();
        continue;
    }

    // Read Head
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_.head), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    auto t = std::chrono::steady_clock::now();

    // Read Body
    size_t body_size = sizeof(rx_data_) - sizeof(rx_data_.head);
    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          body_size)) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (queue_.size() >= queue_capacity_) queue_.pop_front();
        queue_.push_back({q, t});
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.yaw = rx_data_.yaw;
        state_.yaw_vel = rx_data_.yaw_vel;
        state_.pitch = rx_data_.pitch;
        state_.pitch_vel = rx_data_.pitch_vel;
        state_.bullet_speed = rx_data_.bullet_speed;
        state_.bullet_count = rx_data_.bullet_count;
//映射模式
        switch (rx_data_.mode) {
          case 0: mode_ = GimbalMode::IDLE; break;
          case 1: mode_ = GimbalMode::AUTO_AIM; break;
          case 2: mode_ = GimbalMode::SMALL_BUFF; break;
          case 3: mode_ = GimbalMode::BIG_BUFF; break;
          default:
            mode_ = GimbalMode::IDLE;
            tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
            break;
        }
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      if (serial_.IsOpen()) serial_.Close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      open_serial();
      
      {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          queue_.clear();
      }
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
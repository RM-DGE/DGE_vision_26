#ifndef IO__SERIALPORT_MULTITHREAD_HPP
#define IO__SERIALPORT_MULTITHREAD_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#include <functional>
#include <libserial/SerialPort.h>
#include <iostream>

// Include the logger definition
#include "tools/logger.hpp"

// Protocol Headers
constexpr uint8_t IMU_HEADER = 0xAA;
constexpr uint8_t STATUS_HEADER = 0xBB;
constexpr uint8_t CONTROL_HEADER = 0xCC;
constexpr uint8_t IMU_TAIL = 0xDD;

namespace io
{

class SerialPortMultithread
{
public:
    SerialPortMultithread(const std::string& port, int baudrate, 
                         std::function<void(const std::vector<uint8_t>&)> rx_handler)
        : port_(port), baudrate_(baudrate), rx_handler_(rx_handler),
          quit_(false), connected_(false), write_pending_(false)
    {
        // Try connecting immediately
        try_connect();
        
        // Start threads
        read_thread_ = std::thread(&SerialPortMultithread::read_loop, this);
        write_thread_ = std::thread(&SerialPortMultithread::write_loop, this);
        
        tools::logger()->info("SerialPortMultithread initialized on {}", port_);
    }

    ~SerialPortMultithread()
    {
        quit_ = true;
        write_cv_.notify_one();
        
        if (read_thread_.joinable()) read_thread_.join();
        if (write_thread_.joinable()) write_thread_.join();
        
        close_port();
        tools::logger()->info("SerialPortMultithread destroyed");
    }

    void write(const std::vector<uint8_t>& data)
    {
        if (!connected_.load() || quit_.load()) return;

        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            write_queue_.push_back(data);
            write_pending_ = true;
        }
        write_cv_.notify_one();
    }

    struct Statistics {
        size_t frames_received = 0;
        size_t frames_sent = 0;
        size_t read_errors = 0;
        size_t write_errors = 0;
    };
    
    Statistics get_stats() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return stats_;
    }

private:
    std::string port_;
    int baudrate_;
    std::atomic<bool> quit_;
    std::atomic<bool> connected_;
    std::atomic<bool> write_pending_;
    
    LibSerial::SerialPort serial_port_;
    
    std::thread read_thread_;
    std::thread write_thread_;
    
    std::mutex write_mutex_;
    std::condition_variable write_cv_;
    std::deque<std::vector<uint8_t>> write_queue_;
    
    mutable std::mutex stats_mutex_;
    Statistics stats_;
    
    std::function<void(const std::vector<uint8_t>&)> rx_handler_;

    LibSerial::BaudRate get_baud_rate_enum(int baud) {
        switch (baud) {
            case 9600: return LibSerial::BaudRate::BAUD_9600;
            case 115200: return LibSerial::BaudRate::BAUD_115200;
            default: return LibSerial::BaudRate::BAUD_115200;
        }
    }

    void read_loop()
    {
        std::vector<uint8_t> frame_buffer;
        auto last_health_check = std::chrono::steady_clock::now();
        
        while (!quit_) {
            if (!connected_.load()) {
                try_reconnect();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            try {
                size_t available = 0;
                {
                    std::lock_guard<std::mutex> lock(write_mutex_);
                    if (serial_port_.IsOpen()) {
                        available = serial_port_.GetNumberOfBytesAvailable();
                    }
                }

                if (available > 0) {
                    LibSerial::DataBuffer data;
                    {
                        std::lock_guard<std::mutex> lock(write_mutex_);
                        try {
                            serial_port_.Read(data, available, 10); // 10ms timeout
                        } catch (...) {
                            // Ignore read timeouts
                        }
                    }
                    
                    std::vector<uint8_t> vec_data(data.begin(), data.end());
                    process_received_data(vec_data, frame_buffer);
                    
                    {
                        std::lock_guard<std::mutex> lock(stats_mutex_);
                        stats_.read_errors = 0;
                    }
                    last_health_check = std::chrono::steady_clock::now();
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

            } catch (const std::exception& e) {
                tools::logger()->warn("Serial read error: {}", e.what());
                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    stats_.read_errors++;
                }
                connected_ = false;
            }
        }
    }

    void write_loop()
    {
        while (!quit_) {
            std::vector<uint8_t> data_to_send;
            {
                std::unique_lock<std::mutex> lock(write_mutex_);
                write_cv_.wait(lock, [this]() { return write_pending_ || quit_; });
                if (quit_) break;
                
                if (!write_queue_.empty()) {
                    data_to_send = std::move(write_queue_.front());
                    write_queue_.pop_front();
                    write_pending_ = !write_queue_.empty();
                }
            }
            
            if (!data_to_send.empty() && connected_.load()) {
                send_data(data_to_send);
            }
            
            if (write_queue_.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void send_data(const std::vector<uint8_t>& data)
    {
        try {
            std::lock_guard<std::mutex> lock(write_mutex_);
            if (serial_port_.IsOpen()) {
                serial_port_.Write(data);
                serial_port_.DrainWriteBuffer();
                
                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    stats_.frames_sent++;
                }
            }
        } catch (const std::exception& e) {
            tools::logger()->error("Write failed: {}", e.what());
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.write_errors++;
            }
            connected_ = false;
        }
    }

    void process_received_data(const std::vector<uint8_t>& data, std::vector<uint8_t>& frame_buffer)
    {
        // 1. 将新数据放入缓冲区
        for (uint8_t byte : data) {
            frame_buffer.push_back(byte);
        }

        // 2. 动态滑动窗口解析
        while (!frame_buffer.empty()) {
            uint8_t header = frame_buffer[0];

            if (header == 0xAA) {
                // IMU 帧：总长度 10 字节 (带 0xDD 尾)
                if (frame_buffer.size() < 10) break; 
                
                if (frame_buffer[9] == 0xDD) { // 严格校验帧尾防沾包
                    std::vector<uint8_t> complete_frame(frame_buffer.begin(), frame_buffer.begin() + 10);
                    {
                        std::lock_guard<std::mutex> lock(stats_mutex_);
                        stats_.frames_received++;
                    }
                    rx_handler_(complete_frame);
                    frame_buffer.erase(frame_buffer.begin(), frame_buffer.begin() + 10);
                } else {
                    frame_buffer.erase(frame_buffer.begin()); // 假帧头，丢弃
                }
            } 
            else if (header == 0xBB) {
                // 状态帧：总长度 9 字节 (无尾)
                if (frame_buffer.size() < 9) break; 
                
                std::vector<uint8_t> complete_frame(frame_buffer.begin(), frame_buffer.begin() + 9);
                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    stats_.frames_received++;
                }
                rx_handler_(complete_frame);
                frame_buffer.erase(frame_buffer.begin(), frame_buffer.begin() + 9);
            }
            else if (header == 0x5A) {
                // ROS 雷达包：总长度 29 字节 (带 0x0D 尾)
                if (frame_buffer.size() < 29) break;
                
                if (frame_buffer[28] == 0x0D) {
                    // CBoard 不处理底盘数据，直接安全地把这 29 字节整体丢弃！
                    // 这将完美防止它的浮点数数据干扰后面的 AA 或 BB 帧
                    frame_buffer.erase(frame_buffer.begin(), frame_buffer.begin() + 29);
                } else {
                    frame_buffer.erase(frame_buffer.begin());
                }
            }
            else {
                // 遇到其他乱码（包括 0xCC），因为接收端不该收到 0xCC，直接丢弃单字节并滑动
                frame_buffer.erase(frame_buffer.begin());
            }
        }

        if (frame_buffer.size() > 256) frame_buffer.clear();
    }

    bool is_valid_header(uint8_t header)
    {
        return header == IMU_HEADER || header == STATUS_HEADER || header == CONTROL_HEADER;
    }

    void try_connect()
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (serial_port_.IsOpen()) serial_port_.Close();

        try {
            serial_port_.Open(port_);
            serial_port_.SetBaudRate(get_baud_rate_enum(baudrate_));
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            
            connected_ = true;
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_ = Statistics{};
            }
            tools::logger()->info("Serial port connected: {} @ {} baud", port_, baudrate_);
        } catch (const std::exception& e) {
            tools::logger()->warn("Failed to connect to {}: {}", port_, e.what());
            connected_ = false;
        }
    }

    void try_reconnect()
    {
        static auto last_attempt = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_attempt).count() < 2000) return;
        last_attempt = now;
        try_connect();
    }

    void close_port()
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (serial_port_.IsOpen()) {
            serial_port_.Close();
        }
        connected_ = false;
        write_queue_.clear();
        write_pending_ = false;
    }
};

} // namespace io
#endif
#include "usbcamera.hpp"

#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace io
{
USBCamera::USBCamera(const std::string & open_name, const std::string & config_path)
: open_name_(open_name), quit_(false), ok_(false), queue_(1), open_count_(0)
{
  auto yaml = tools::load(config_path);
  image_width_ = tools::read<double>(yaml, "image_width");
  image_height_ = tools::read<double>(yaml, "image_height");
  usb_exposure_ = tools::read<double>(yaml, "usb_exposure");
  usb_frame_rate_ = tools::read<double>(yaml, "usb_frame_rate");
  usb_gamma_ = tools::read<double>(yaml, "usb_gamma");
  usb_gain_ = tools::read<double>(yaml, "usb_gain");
  try_open();

  // 守护线程
  daemon_thread_ = std::thread{[this] {
    // tools::logger()->info("daemon thread start");
    while (!quit_) {
      std::this_thread::sleep_for(100ms);

      if (ok_) continue;

      if (open_count_ > 20) {
        tools::logger()->warn("Give up to open {} USB camera", this->device_name);
        quit_ = true;

        {
          std::lock_guard<std::mutex> lock(cap_mutex_);
          close();  // 先关闭摄像头
        }

        if (capture_thread_.joinable()) {
          tools::logger()->warn("Stopping capture thread");
          capture_thread_.join();
        }

        break;
      }

      if (capture_thread_.joinable()) capture_thread_.join();

      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        close();
      }
      try_open();
    }
    // tools::logger()->info("daemon thread exit");
  }};
}

USBCamera::~USBCamera()
{
  quit_ = true;
  {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    close();
  }
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  tools::logger()->info("USBCamera destructed.");
}

cv::Mat USBCamera::read()
{
  std::lock_guard<std::mutex> lock(cap_mutex_);
  if (!cap_.isOpened()) {
    tools::logger()->warn("Failed to read {} USB camera", this->device_name);
    return cv::Mat();
  }
  cap_ >> img_;
  return img_;
}

void USBCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void USBCamera::open()
{
  std::lock_guard<std::mutex> lock(cap_mutex_);

  // 允许传入 "video2"、"v4l/by-path/..." 或完整的 "/dev/..." 路径。
  // 推荐使用 /dev/v4l/by-path 下的稳定路径，避免插拔后 videoN 变化。
  std::string true_device_name = open_name_;
  if (true_device_name.rfind("/dev/", 0) != 0) {
    true_device_name = "/dev/" + true_device_name;
  }

  cap_.open(true_device_name, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    tools::logger()->warn("Failed to open USB camera {}", true_device_name);
    return;
  }
  sharpness_ = cap_.get(cv::CAP_PROP_SHARPNESS);
  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  // 先定分辨率，再定帧率；否则部分 UVC 相机会把 FPS 重置回默认值。
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
  cap_.set(cv::CAP_PROP_FPS, usb_frame_rate_);
  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
  cap_.set(cv::CAP_PROP_GAMMA, usb_gamma_);
  cap_.set(cv::CAP_PROP_GAIN, usb_gain_);
  cap_.set(cv::CAP_PROP_EXPOSURE, usb_exposure_);

  // 部分相机可用 sharpness 区分左右（2=左，3=右）。
  // 当前 RYS USB Camera 上报的是 80，因此名称由调用方显式指定；
  // 分辨率/曝光不再依赖 sharpness 判断，统一按 YAML 应用。
  if (sharpness_ == 2) {
    device_name = "left";
  } else if (sharpness_ == 3) {
    device_name = "right";
  }

  const auto & display_name = device_name.empty() ? open_name_ : device_name;
  tools::logger()->info(
    "{} USBCamera opened: {} {}x{}@{:.0f}fps ex={}", display_name, true_device_name,
    static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
    static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)), cap_.get(cv::CAP_PROP_FPS),
    cap_.get(cv::CAP_PROP_EXPOSURE));

  // 取图线程
  capture_thread_ = std::thread{[this] {
    ok_ = true;
    std::this_thread::sleep_for(50ms);
    tools::logger()->info("[{} USB camera] capture thread started ", this->device_name);
    while (!quit_) {
      std::this_thread::sleep_for(1ms);

      cv::Mat img;
      bool success;
      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        if (!cap_.isOpened()) {
          break;
        }
        success = cap_.read(img);
      }

      if (!success) {
        tools::logger()->warn("Failed to read frame, exiting capture thread");
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      queue_.push({img, timestamp});
    }
    ok_ = false;
  }};
}

void USBCamera::try_open()
{
  try {
    open();
    open_count_++;
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void USBCamera::close()
{
  if (cap_.isOpened()) {
    cap_.release();
    tools::logger()->info("USB camera released.");
  }
}

}  // namespace io
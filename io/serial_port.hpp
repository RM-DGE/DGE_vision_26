#ifndef IO_SERIAL_PORT_HPP
#define IO_SERIAL_PORT_HPP

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace io
{
// Linux 8N1 serial descriptor. Closing a removed device must release the fd even
// when terminal attributes can no longer be restored. The wire protocol and the
// receive loop's host timestamp location remain in SerialPortMultithread/CBoard.
class SerialPort
{
public:
  ~SerialPort() { Close(); }
  SerialPort() = default;
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  bool IsOpen() const { return fd_ >= 0; }
  void Open(const std::string & path, int baud)
  {
    if (IsOpen()) throw std::runtime_error("serial port already open");
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) error("open serial");
    try {
      if (::ioctl(fd_, TIOCEXCL) < 0) error("exclusive serial access");
      termios settings{};
      if (::tcgetattr(fd_, &settings) < 0) error("read serial settings");
      ::cfmakeraw(&settings);
      settings.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS | CSIZE);
      settings.c_cflag |= CS8 | CREAD | CLOCAL;
      settings.c_cc[VMIN] = 0;
      settings.c_cc[VTIME] = 0;
      // Match the project's previous supported baud rates and fallback.
      const speed_t speed = baud == 9600 ? B9600 : B115200;
      if (
        ::cfsetispeed(&settings, speed) < 0 || ::cfsetospeed(&settings, speed) < 0 ||
        ::tcsetattr(fd_, TCSANOW, &settings) < 0 || ::tcflush(fd_, TCIOFLUSH) < 0)
        error("configure serial");
    } catch (...) {
      Close();
      throw;
    }
  }
  void Close() noexcept
  {
    if (!IsOpen()) return;
    const int old = fd_;
    fd_ = -1;
    ::ioctl(old, TIOCNXCL);  // Best effort; ENODEV/EIO after unplug is expected.
    ::close(old);            // Do not retry close: the descriptor may already be released.
  }
  std::size_t GetNumberOfBytesAvailable() const
  {
    int count = 0;
    if (::ioctl(fd_, FIONREAD, &count) < 0) error("read serial availability");
    return count > 0 ? static_cast<std::size_t>(count) : 0;
  }
  void Read(std::vector<std::uint8_t> & data, std::size_t count, int timeout_ms)
  {
    data.clear();
    data.reserve(count);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::uint8_t bytes[4096];
    while (data.size() < count) {
      const auto remaining = count - data.size();
      const auto n = ::read(fd_, bytes, remaining < sizeof(bytes) ? remaining : sizeof(bytes));
      if (n > 0) {
        data.insert(data.end(), bytes, bytes + n);
        continue;
      }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) error("read serial");
      if (!wait(POLLIN, deadline)) return;  // Preserve any partial packet for the parser.
    }
  }
  void Write(const std::vector<std::uint8_t> & data)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    std::size_t sent = 0;
    while (sent < data.size()) {
      const auto n = ::write(fd_, data.data() + sent, data.size() - sent);
      if (n > 0) {
        sent += n;
        continue;
      }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) error("write serial");
      if (!wait(POLLOUT, deadline)) throw std::runtime_error("serial write timeout");
    }
  }
  void DrainWriteBuffer()
  {
    int result;
    do {
      result = ::tcdrain(fd_);
    } while (result < 0 && errno == EINTR);
    if (result < 0) error("drain serial");
  }

private:
  int fd_ = -1;
  [[noreturn]] static void error(const char * operation)
  {
    throw std::system_error(errno, std::generic_category(), operation);
  }
  bool wait(short event, std::chrono::steady_clock::time_point deadline)
  {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - std::chrono::steady_clock::now())
                        .count();
    if (left < 0) return false;
    pollfd item{fd_, event, 0};
    const int result = ::poll(&item, 1, static_cast<int>(left + 1));
    if (result < 0) {
      if (errno == EINTR) return true;
      error("poll serial");
    }
    if (item.revents & (POLLERR | POLLHUP | POLLNVAL))
      throw std::runtime_error("serial disconnected");
    return result > 0;
  }
};
}  // namespace io
#endif

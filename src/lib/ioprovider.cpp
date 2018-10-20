#include "ics-servo/ioprovider.h"

#include <stdexcept>

namespace IcsServo {

IOProvider::IOProvider(std::string const& device, speed_t speed, std::size_t en_idx)
  : en_pin_idx(en_idx) {
    const int serial_fd = ::open(device.c_str(), O_RDWR);
    if (serial_fd < 0) {
      throw std::runtime_error("Cannot open " + device);
    }

    termios tio;
    tio.c_cflag += CREAD;
    tio.c_cflag += CLOCAL;
    tio.c_cflag += CS8;

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    cfmakeraw(&tio);

    tcsetattr(serial_fd, TCSANOW, &tio);

    if(ioctl(serial_fd, TCSETS, &tio) < 0) {
      throw std::runtime_error("Cannot set serial port setting to " + device);
    }
    ::close(serial_fd);
    this->serial_stream.open(device);

    auto const export_fd = ::open("/sys/class/gpio/export", O_RDWR);
    if(export_fd < 0) {
      throw std::runtime_error("Cannot open /sys/class/gpio/export");
    }

    char buf[16];
    ::sprintf(buf, "%d\n", this->en_pin_idx);
    if(::write(export_fd, buf, std::strlen(buf)) < 0) {
      throw std::runtime_error("Cannot write on /sys/class/gpio/export");
    }
    ::close(export_fd);

    std::string const gpio_base = "/sys/class/gpio/gpio" + std::to_string(this->en_pin_idx);
    auto const direction_path = gpio_base + "/direction";
    auto const direction_fd = ::open(direction_path.c_str(), O_RDWR);
    if(direction_fd < 0) {
      throw std::runtime_error("Cannot open " + direction_path);
    }
    if(::write(direction_fd, "out\n", std::strlen(4)) < 0) {
      throw std::runtime_error("Cannot write on " + direction_path);
    }
    ::close(direction_fd);

    auto const value_path = gpio_base + "/value";
    this->gpio_fd = ::open(value_path.c_str(), O_RDWR);
    if(this->value_fd < 0) {
      throw std::runtime_error("Cannot open " + value_path);
    }
}

void IOProvider::set_gpio_value(bool state) {
  if(::write(this->gpio_fd, state ? "1\n" : "0\n", std::strlen(2)) < 0) {
    throw std::runtime_error("Cannot write gpio value ");
  }
}

}

#include "ics-servo/ics.h"

#include <stdexcept>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace ICSServo {

IOProvider::IOProvider(std::string const& device, std::size_t en_idx_, speed_t speed)
  : en_idx(en_idx_), is_closed(false)
  {
    this->serial_fd = ::open(device.c_str(), O_RDWR);
    if (this->serial_fd < 0) {
      throw std::runtime_error("Cannot open " + device);
    }

    auto const export_fd = ::open("/sys/class/gpio/export", O_RDWR);
    if(export_fd < 0) {
      throw std::runtime_error("Cannot open /sys/class/gpio/export");
    }

    char buf[16];
    ::sprintf(buf, "%d\n", this->en_idx);
    if(::write(export_fd, buf, std::strlen(buf)) < 0) {
      throw std::runtime_error("Cannot write on /sys/class/gpio/export");
    }
    ::close(export_fd);

    std::string const gpio_base = "/sys/class/gpio/gpio" + std::to_string(this->en_idx);
    auto const direction_path = gpio_base + "/direction";
    auto const direction_fd = ::open(direction_path.c_str(), O_RDWR);
    if(direction_fd < 0) {
      throw std::runtime_error("Cannot open " + direction_path);
    }
    if(::write(direction_fd, "out\n", 4) < 0) {
      throw std::runtime_error("Cannot write on " + direction_path);
    }
    ::close(direction_fd);

    auto const value_path = gpio_base + "/value";
    this->gpio_fd = ::open(value_path.c_str(), O_RDWR);
    if(this->gpio_fd < 0) {
      throw std::runtime_error("Cannot open " + value_path);
    }
}

IOProvider::~IOProvider() {
  try {
    try {
      this->close();
    } catch (const std::exception& e) {
      std::cerr << "Exeption happend during the destruction of IOProvider" << std::endl << e.what() << std::endl;
      std::abort();
    }
  } catch (...) {}
}

void IOProvider::close() {
  if (!this->is_closed) {
    ::close(this->gpio_fd);
    ::close(this->serial_fd);
    auto const export_fd = ::open("/sys/class/gpio/unexport", O_RDWR);
    if(export_fd < 0) {
      throw std::runtime_error("Cannot open /sys/class/gpio/unexport");
    }

    char buf[16];
    ::sprintf(buf, "%d\n", this->en_idx);
    if(::write(export_fd, buf, std::strlen(buf)) < 0) {
      throw std::runtime_error("Cannot write on /sys/class/gpio/unexport");
    }
    ::close(export_fd);
    this->is_closed = true;
  }
}

void IOProvider::set_gpio_value(bool state) {
  if(::write(this->gpio_fd, state ? "1\n" : "0\n", 2) < 0) {
    throw std::runtime_error("Cannot write gpio value ");
  }
}

void IOProvider::set_id(ServoID id_in) {
  std::uint8_t command[4] = {
    static_cast<std::uint8_t>(0xE0 + (0x1F & id_in)),
    0x01,
    0x01,
    0x01,
  };

  this->send(std::cbegin(command), std::cend(command));
}

ServoID IOProvider::get_id() {
  std::uint8_t command[4] = {
    0xFF,
    0x00,
    0x00,
    0x00,
  };

  this->send(std::cbegin(command), std::cend(command));
  std::vector<std::uint8_t> recv_buf(5);
  this->recv(5, std::begin(recv_buf));
  ServoID id_recv = recv_buf[4] & 0x1F;
  return id_recv;
}

void IOProvider::write_serial(std::uint8_t const* ptr, std::size_t len) {
  if(::write(this->serial_fd, ptr, len) < 0) {
    throw std::runtime_error("Cannot write to serial device file");
  }
}

}

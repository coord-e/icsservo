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

#ifdef B115200
#define HAS_B115200
#else
// serial_struct, ASYNC_SPD_VHI
#include <linux/serial.h>
#endif

namespace ICSServo {

void IOProvider::init_serial(std::string const& device) {
    this->serial_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (this->serial_fd < 0) {
      throw std::runtime_error("Cannot open " + device);
    }

    if (tcgetattr(this->serial_fd, &this->prev_term_config) < 0) {
      throw std::runtime_error("Cannot get serial port configuration from " + device);
    }

    termios conf;
    conf.c_iflag = 0;
    conf.c_oflag &= ~static_cast<unsigned>(OPOST);
    conf.c_lflag &= ~static_cast<unsigned>(ISIG | ICANON | TOSTOP | FLUSHO | NOFLSH | XCASE | EXTPROC);
    conf.c_cflag |= CS8 | CREAD | CLOCAL | PARENB;
    conf.c_cflag &= ~static_cast<unsigned>(PARODD | CMSPAR);

    conf.c_cc[VMIN] = 1;
    conf.c_cc[VTIME] = 0;

#ifdef HAS_B115200
    const auto baud = B115200;
#else
    // In an environment where there is no B115200,
    // We have to set spd_vhi flag to use 115200 instead of 38400

    const auto baud = B38400;

    serial_struct serinfo;
    if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
      throw std::runtime_error("Cannot get serial port configuration (TIOCGSERIAL) from " + device);
    }
    serinfo.flags |= ASYNC_SPD_VHI;
    if (ioctl(this->serial_fd, TIOCSSERIAL, &serinfo) < 0) {
      throw std::runtime_error("Cannot set serial port configuration (TIOCSSERIAL) to " + device);
    }
#endif

    cfsetispeed(&conf, baud);
    cfsetospeed(&conf, baud);

    if (tcsetattr(this->serial_fd, TCSANOW, &conf) < 0) {
      throw std::runtime_error("Cannot set serial port configuration to " + device);
    }

    termios new_conf;
    if (tcgetattr(this->serial_fd, &new_conf) < 0) {
      throw std::runtime_error("Cannot get serial port configuration from " + device);
    }

    // Not comparing the entire c_cc, because it differs in my environtment...
    // That's also a reason why memcmp isn't used here
    if (conf.c_iflag != new_conf.c_iflag ||
        conf.c_oflag != new_conf.c_oflag ||
        conf.c_cflag != new_conf.c_cflag ||
        conf.c_lflag != new_conf.c_lflag ||
        conf.c_cc[VMIN] != new_conf.c_cc[VMIN] ||
        conf.c_cc[VTIME] != new_conf.c_cc[VTIME] ||
        conf.c_ispeed != new_conf.c_ispeed ||
        conf.c_ospeed != new_conf.c_ospeed) {
      throw std::runtime_error("Cannot set all serial port configuration to " + device);
    }
}

void IOProvider::init_gpio_export() {
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
}

void IOProvider::init_gpio_setup() {
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

    if (tcsetattr(this->serial_fd, TCSANOW, &this->prev_term_config) < 0) {
      throw std::runtime_error("Cannot restore serial port configuration");
    }
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

  this->send(command, 4);
}

ServoID IOProvider::get_id() {
  std::uint8_t command[4] = {
    0xFF,
    0x00,
    0x00,
    0x00,
  };

  this->send(command, 4);
  std::vector<std::uint8_t> recv_buf(5);
  this->recv(recv_buf.data(), 5);
  ServoID id_recv = recv_buf[4] & 0x1F;
  return id_recv;
}

void IOProvider::write_serial(std::uint8_t const* ptr, std::size_t len) {
  if(::write(this->serial_fd, ptr, len) < 0) {
    throw std::runtime_error("Cannot write to serial device file");
  }
}

void IOProvider::read_serial(std::uint8_t* ptr, std::size_t len) {
  if(::read(this->serial_fd, ptr, len) < 0) {
    throw std::runtime_error("Cannot read from serial device file");
  }
}

}

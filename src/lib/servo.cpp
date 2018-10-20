#include "ics-servo/servo.h"

#include <stdexcept>

namespace IcsServo {

UARTProvider::UARTProvider(std::string const& device, speed_t speed, std::size_t en_idx)
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

void UARTProvider::set_gpio_value(bool state) {
  if(::write(this->gpio_fd, state ? "1\n" : "0\n", std::strlen(2)) < 0) {
    throw std::runtime_error("Cannot write gpio value ");
  }
}

InternalPosition Servo::rad_to_internal(Position pos) {
  const double deg = pos * 180 / M_PI;
  return deg * 29.633 + 7500;
}

Position Servo::internal_to_rad(InternalPosition ipos) {
  const double deg = (ipos - 7500) / 29.633;
  return deg * M_PI / 180;
}

bool Servo::check_range(Position) {
  return pos <= Servo::max_pos || pos >= Servo::min_pos;
}

void Servo::write_param(Subcommand sc, std::uint8_t data) {
  if (data < 1 || data > 127) {
    throw std::out_of_range("Parameter value out of range.");
  }

  std::uint8_t command[3] = {
    0xC0 + this->id,
    static_cast<std::uint8_t>(sc),
    data
  };

  this->provider->send(std::cbegin(command), std::cend(command));
}

std::uint8_t Servo::read_param(Subcommand sc) {
  std::uint8_t command[2] = {
    0xA0 + this->id,
    static_cast<std::uint8_t>(sc)
  };

  this->provider->send(std::cbegin(command), std::cend(command));
  std::vector<std::uint8_t> recv(3);
  this->provider->read(3, std::begin(recv));
  return recv[3];
}

Servo::Servo(std::shared_ptr<UARTProvider> prov_, ServoID id_) : provider(prov_), id(id_) {}

void Servo::set_position(Position pos) {
  if (!this->check_range(pos)) {
    throw std::out_of_range("Position out of range.");
  }

  auto const ipos = this->rad_to_internal(pos);
  std::uint8_t command[3] = {
    0x80 + this->id,
    (ipos >> 7) & 0x007F,
    ipos & 0x007F
  };

  this->provider->send(std::cbegin(command), std::cend(command));
}

void Servo::set_free() {
  std::uint8_t command[3] = {
    0x80 + this->id,
    0,
    0
  };

  this->provider->send(std::cbegin(command), std::cend(command));
}

void Servo::set_stretch(std::uint8_t stretch) {
  this->write_param(Subcommand::STRC, stretch);
}

void Servo::set_speed(std::uint8_t speed) {
  this->write_param(Subcommand::SPD, speed);
}

void Servo::set_current_limit(std::uint8_t current_limit) {
  this->write_param(Subcommand::CUR, current_limit);
}

void Servo::set_temperature_limit(std::uint8_t tmperature_limit) {
  this->write_param(Subcommand::TMP, temperature_limit);
}

std::uint8_t Servo::get_stretch() {
  return this->read_param(Subcommand::STRC);
}

std::uint8_t Servo::get_speed() {
  return this->read_param(Subcommand::SPD);
}

std::uint8_t Servo::get_current() {
  return this->read_param(Subcommand::CUR);
}

std::uint8_t Servo::get_temperature() {
  return this->read_param(Subcommand::TMP);
}

/* Position Servo::get_position() { */
/*   return this->read_param(Subcommand::TCH); */
/* } */

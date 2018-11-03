#include "ics-servo/servo.h"

#include <stdexcept>
#include <vector>

namespace ICSServo {

Servo::InternalPosition Servo::rad_to_internal(Servo::Position pos) {
  const double deg = pos * 180 / M_PI;
  return deg * 29.633 + 7500;
}

Servo::Position Servo::internal_to_rad(Servo::InternalPosition ipos) {
  const double deg = (ipos - 7500) / 29.633;
  return deg * M_PI / 180;
}

bool Servo::check_range(Servo::Position pos) {
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

Servo::Servo(std::shared_ptr<IOProvider> prov_, ServoID id_) : provider(prov_), id(id_) {}

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

Position Servo::get_position() {
  std::uint8_t command[2] = {
    0xA0 + this->id,
    static_cast<std::uint8_t>(Subcommand::TCH)
  };

  this->provider->send(std::cbegin(command), std::cend(command));
  std::vector<std::uint8_t> recv(3);
  this->provider->read(3, std::begin(recv));
  InternalPosition ipos = ((recv[2] << 7) & 0x3F80) + (recv[3] & 0x007F);
  return this->internal_to_rad(ipos);
}

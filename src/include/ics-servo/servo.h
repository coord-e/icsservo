#ifndef ICS_SERVO_SERVO_H
#define ICS_SERVO_SERVO_H

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <memory>

#include "ics-servo/ics.h"

namespace ICSServo {

template <class T>
static constexpr T pi = static_cast<T>(3.141592653589793238462643383279502884e+00);

enum class Subcommand {
  EEPROM = 0x00,
  STRC = 0x01,
  SPD = 0x02,
  CUR = 0x03,
  TMP = 0x04,
  TCH = 0x05
};

class Servo {
  std::shared_ptr<IOProvider> provider;
  ServoID id;

public:
  Servo(std::shared_ptr<IOProvider>, ServoID);

  using Position = double;
  static constexpr Position max_pos = pi<Position>;
  static constexpr Position min_pos = -pi<Position>;

  void set_position(Position);
  void set_free();

  void set_stretch(std::uint8_t stretch);
  void set_speed(std::uint8_t speed);
  void set_current_limit(std::uint8_t current_limit);
  void set_temperature_limit(std::uint8_t tmperature_limit);

  std::uint8_t get_stretch();
  std::uint8_t get_speed();
  std::uint8_t get_current();
  std::uint8_t get_temperature();
  Position get_position();

private:
  using InternalPosition = std::uint16_t;
  InternalPosition rad_to_internal(Position);
  Position internal_to_rad(InternalPosition);
  bool check_range(Position);

  void write_param(Subcommand sc, std::uint8_t data);
  std::uint8_t read_param(Subcommand sc);
};

}

#endif

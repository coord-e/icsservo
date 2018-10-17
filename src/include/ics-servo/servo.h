#ifndef ICS_SERVO_SERVO_H
#define ICS_SERVO_SERVO_H

#include <cstddef>
#include <cstdint>

namespace IcsServo {

using ServoID = std::uint8_t;

class UARTProvider {
public:
  template<typename InputIterator, typename OutputIterator>
  void send_and_recv(InputIterator first, InputIterator last, std::size_t n, OutputIterator result);
};

class Servo {
  ServoID id;

public:
  Servo(UARTProvider, ServoID);

  using Position = double;

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
};

}

#endif

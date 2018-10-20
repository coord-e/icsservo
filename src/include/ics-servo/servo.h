#ifndef ICS_SERVO_SERVO_H
#define ICS_SERVO_SERVO_H

#include <cstddef>
#include <cstdint>

namespace IcsServo {

using ServoID = std::uint8_t;

class UARTProvider {
  std::fstream serial_stream;
  int gpio_fd;

public:
  UARTProvider(std::string const& device, speed_t speed, std::size_t en_pin_idx);

  template<typename InputIterator>
  void send(InputIterator first, InputIterator last) {
    this->set_gpio_value(true); // send
    std::copy(first, last, std::ostreambuf_iterator<std::uint8_t>(this->serial_stream));
    this->serial_stream.rdbuf()->sync();
  }

  template<typename OutputIterator>
  void recv(std::size_t n, OutputIterator first) {
    this->set_gpio_value(false); // recv
    std::copy(std::istreambuf_iterator<std::uint8_t>(this->serial_stream), n, first);
    this->serial_stream.rdbuf()->sync();
  }

private:
  void set_gpio_value(bool state);
};


enum class Subcommand {
  EEPROM = 0x00,
  STRC = 0x01,
  SPD = 0x02,
  CUR = 0x03,
  TMP = 0x04,
  TCH = 0x05
};

class Servo {
  ServoID id;
  std::shared_ptr<UARTProvider> provider;

public:
  Servo(std::shared_ptr<UARTProvider>, ServoID);

  using Position = double;
  static constexpr Position max_pos = M_PI;
  static constexpr Position min_pos = -M_PI;

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

#ifndef ICS_SERVO_SERVO_H
#define ICS_SERVO_SERVO_H

#include <cstddef>
#include <cstdint>

namespace IcsServo {

using ServoID = std::uint8_t;

class UARTProvider {
  int serial_fd;
  int gpio_fd;

public:
  UARTProvider(std::string const& device, speed_t speed, std::size_t en_pin_idx);

  template<typename InputIterator, typename OutputIterator>
  void send_and_recv(InputIterator first, InputIterator last, std::size_t n, OutputIterator result) {
    this->set_gpio_value(true); // send
    if(::write(this->serial_fd, first, std::distance(first, last)) < 0) {
      throw std::runtime_error("Cannot write serial");
    }

    this->set_gpio_value(false); // recv

    std::vector<std::uint8_t> v(n);
    if(::read(this->serial_fd, v.data(), n) < 0) {
      throw std::runtime_error("Cannot read serial");
    }

    std::copy_n(std::cbegin(v), n, result);
  }

private:
  void set_gpio_value(bool state);
};

class Servo {
  ServoID id;
  std::shared_ptr<UARTProvider> provider;

public:
  Servo(std::shared_ptr<UARTProvider>, ServoID);

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

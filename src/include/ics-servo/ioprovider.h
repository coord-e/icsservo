#ifndef ICS_SERVO_IOPROVIDER_H
#define ICS_SERVO_IOPROVIDER_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <iterator>
#include <termios.h>
#include <algorithm>

namespace ICSServo {

class IOProvider {
  std::basic_fstream<std::uint8_t> serial_stream;
  int gpio_fd;

public:
  IOProvider(std::string const& device, speed_t speed, std::size_t en_pin_idx);

  template<typename InputIterator>
  void send(InputIterator first, InputIterator last) {
    this->set_gpio_value(true); // send
    std::copy(first, last, std::ostreambuf_iterator<std::uint8_t>(this->serial_stream));
    this->serial_stream.flush();
  }

  template<typename OutputIterator>
  void recv(std::size_t n, OutputIterator first) {
    this->set_gpio_value(false); // recv
    std::copy_n(std::istreambuf_iterator<std::uint8_t>(this->serial_stream), n, first);
  }

private:
  void set_gpio_value(bool state);
};

}

#endif

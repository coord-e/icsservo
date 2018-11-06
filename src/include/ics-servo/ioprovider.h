#ifndef ICS_SERVO_IOPROVIDER_H
#define ICS_SERVO_IOPROVIDER_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <iterator>
#include <termios.h>
#include <algorithm>
#include <vector>

#include "ics-servo/ics.h"

namespace ICSServo {

class IOProvider {
  int gpio_fd, serial_fd;
  std::size_t en_idx;
  bool is_closed;

public:
  // The real baud rate of B38400 can be configured using setserial(8)
  IOProvider(std::string const& device, std::size_t en_pin_idx, speed_t speed = B38400);
  ~IOProvider();

  void close();

  void send(std::uint8_t const* buf, std::size_t len) {
    this->set_gpio_value(true); // send
    this->write_serial(buf, len);
  }

  template<typename InputIterator>
  void send(InputIterator first, InputIterator last) {
    this->set_gpio_value(true); // send

    std::vector<std::uint8_t> buf(first, last);

    this->write_serial(buf.data(), buf.size());
  }

  template<typename OutputIterator>
  void recv(std::size_t n, OutputIterator first) {
    this->set_gpio_value(false); // recv
    // ditto
    std::copy_n(std::istreambuf_iterator<char>(this->serial_stream), n, first);
  }

  void set_id(ServoID);
  ServoID get_id();

private:
  void set_gpio_value(bool state);
  void write_serial(std::uint8_t const* ptr, std::size_t len);
};

}

#endif

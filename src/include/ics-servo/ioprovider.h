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
  IOProvider(std::string const& device, std::size_t en_pin_idx);
  ~IOProvider();

  void close();

  void send(std::uint8_t const* buf, std::size_t len) {
    this->set_gpio_value(true); // send
    this->write_serial(buf, len);
  }

  template<typename InputIterator>
  void send(InputIterator first, InputIterator last) {
    std::vector<std::uint8_t> buf(first, last);

    this->send(buf.data(), buf.size());
  }

  void recv(std::uint8_t* buf, std::size_t len) {
    this->set_gpio_value(false); // recv
    this->read_serial(buf, len);
  }

  template<typename OutputIterator>
  void recv(OutputIterator first, std::size_t len) {
    std::vector<std::uint8_t> buf (len);

    this->recv(buf.data(), len);
  }

  void set_id(ServoID);
  ServoID get_id();

private:
  void set_gpio_value(bool state);
  void write_serial(std::uint8_t const* ptr, std::size_t len);
  void read_serial(std::uint8_t* ptr, std::size_t len);
};

}

#endif

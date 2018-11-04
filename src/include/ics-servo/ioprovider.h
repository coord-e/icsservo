#ifndef ICS_SERVO_IOPROVIDER_H
#define ICS_SERVO_IOPROVIDER_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <iterator>
#include <termios.h>
#include <algorithm>

#include "ics-servo/ics.h"

namespace ICSServo {

class IOProvider {
  std::fstream serial_stream;
  int gpio_fd;
  std::size_t en_idx;
  bool is_closed;

public:
  IOProvider(std::string const& device, std::size_t en_pin_idx, speed_t speed = B115200);
  ~IOProvider();

  void close();

  template<typename InputIterator>
  void send(InputIterator first, InputIterator last) {
    this->set_gpio_value(true); // send
    // This should be ostreambuf_iterator<uint8_t>, however it causes runtime error
    // The code below works where sizeof(char) == 1, otherwise it may cause compilation error
    std::copy(first, last, std::ostreambuf_iterator<char>(this->serial_stream));
    this->serial_stream.flush();
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
  void initialize(std::string const& device, std::size_t en_pin_idx, speed_t speed);
};

}

#endif

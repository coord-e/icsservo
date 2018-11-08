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
#include <chrono>
#include <thread>

#include "ics-servo/ics.h"

namespace ICSServo {

class IOProvider {
  int gpio_fd, serial_fd;
  std::size_t en_idx;
  bool is_closed;
  termios prev_term_config;

public:

  template <class Rep = std::chrono::milliseconds::rep, class Period = std::chrono::milliseconds::period>
  IOProvider(std::string const& device, std::size_t en_idx_, const std::chrono::duration<Rep, Period>& export_delay = std::chrono::milliseconds(100))
    : en_idx(en_idx_), is_closed(false)
  {
    this->init_serial(device);
    this->init_gpio_export();

    std::this_thread::sleep_for(export_delay);

    this->init_gpio_setup();
  }

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
  void init_serial(std::string const&);
  void init_gpio_export();
  void init_gpio_setup();

  void set_gpio_value(bool state);
  void write_serial(std::uint8_t const* ptr, std::size_t len);
  void read_serial(std::uint8_t* ptr, std::size_t len);
};

}

#endif

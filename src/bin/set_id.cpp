#include "ics-servo/ics.h"

#include <iostream>

int main(int argc, char **argv) {

  if (argc != 4) {
    std::cerr << "set_id <device> <en_pin> <id>" << std::endl;
    return 1;
  }

  const std::string device {argv[1]};
  const std::uint8_t en_pin  = static_cast<std::uint8_t>(strtol(argv[2], nullptr, 0));
  const ICSServo::ServoID id  = static_cast<ICSServo::ServoID>(strtol(argv[3], nullptr, 0));

  auto sa = std::make_shared<ICSServo::IOProvider>(device, en_pin);
  sa->set_id(id);
  sa->close();
}

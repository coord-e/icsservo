#include "ics-servo/ics.h"

#include <iostream>

int main(int argc, char **argv) {

  if (argc != 3) {
    std::cerr << "get_id <device> <en_pin>" << std::endl;
    return 1;
  }

  const std::string device {argv[1]};
  const std::uint8_t en_pin  = static_cast<std::uint8_t>(strtol(argv[2], nullptr, 0));

  auto sa = std::make_shared<ICSServo::IOProvider>(device, B115200, en_pin);
  std::cout << static_cast<int>(sa->get_id()) << std::endl;
  sa->close();
}

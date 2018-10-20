#include "ics-servo/servo.h"

#include <iostream>

int main(int argc, char **argv) {

  if (argc != 4) {
    std::cerr << "servo <device> <en_pin> <id>" << std::endl;
    return 1;
  }

  const std::string device {argv[1]};
  const std::uint8_t en_pin  = static_cast<std::uint8_t>(strtol(argv[2], nullptr, 0));
  const IcsServo::ServoID id  = static_cast<IcsServo::ServoID>(strtol(argv[3], nullptr, 0));

  auto sa = std::make_shared<IcsServo::IOProvider>(device, B115200, en_pin);
  auto servo = IcsServo::Servo(sa, id);

  while(true) {
    double rad;
    std::cout << " (rad)  > ";
    std::cin >> rad;

    servo.set_position(rad);
  }
}

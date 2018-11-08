#include "ics-servo/servo.h"

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <cstring>
#include <atomic>

std::atomic<bool> quit(false);    // signal flag

void on_signal(int);
void register_signal(int);

int main(int argc, char **argv) {

  if (argc != 4) {
    std::cerr << "servo <device> <en_pin> <id>" << std::endl;
    return 1;
  }

  const std::string device {argv[1]};
  const std::uint8_t en_pin  = static_cast<std::uint8_t>(strtol(argv[2], nullptr, 0));
  const ICSServo::ServoID id  = static_cast<ICSServo::ServoID>(strtol(argv[3], nullptr, 0));

  auto io = std::make_shared<ICSServo::IOProvider>(device, en_pin);
  auto servo = ICSServo::Servo(io, id);

  register_signal(SIGINT);
  register_signal(SIGTERM);
  register_signal(SIGQUIT);

  while(true) {
    double rad;
    std::cout << " (rad)  > ";
    std::cin >> rad;

    servo.set_position(rad);

    if(quit.load()) break;
  }
}

void on_signal(int)
{
  quit.store(true);
}

void register_signal(int sig) {
  struct sigaction sa;
  memset( &sa, 0, sizeof(sa) );
  sa.sa_handler = on_signal;
  sigfillset(&sa.sa_mask);
  sigaction(sig, &sa, NULL);
}

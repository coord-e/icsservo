#include "ics-servo/ics.h"
#include <servoarray/driver.h>

namespace ICSServo {

class ICSDriver final : public ServoArray::Driver {
  std::vector<Servo> servos_;

public:
  ICSDriver(std::uint8_t size, std::shared_ptr<IOProvider> io) {
    this->servos_.reserve(size);
    for (std::uint8_t i = 0; i < size; i++) {
      this->servos_.push_back(Servo{io, i});
    }
  }

  void write(std::size_t idx, double pos) override {
    this->servos_[idx].set_position(pos);
  }

  double read(std::size_t idx) override {
    return this->servos_[idx].get_position();
  }

  std::size_t size() const override {
    return this->servos_.size();
  }
};

}

extern "C" ServoArray::Driver* servoarray_driver(const ServoArray::DriverParams& params) {
  const double delay = params.get_or<double>("delay", 0.5);
  auto io = std::make_shared<ICSServo::IOProvider>(params.get<std::string>("device"), params.get<unsigned>("en_pin"), std::chrono::duration<double>(delay));
  return new ICSServo::ICSDriver(params.get<std::uint8_t>("size"), io);
}

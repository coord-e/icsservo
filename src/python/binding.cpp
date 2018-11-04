#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "ics-servo/ics.h"

namespace py = pybind11;

namespace Adaptor {

class IOProvider {
  ::ICSServo::IOProvider provider;

public:
  IOProvider(std::string const& device, std::size_t en_pin_idx) : provider(device, en_pin_idx) {}

  void set_id(::ICSServo::ServoID id) {
    this->provider.set_id(id);
  }

  ::ICSServo::ServoID get_id() {
    return this->provider.get_id();
  }
};

}

PYBIND11_MODULE(servoarray, m) {
  m.doc() = "ICSServo: ICS serial servo driver library";
  py::class_<Adaptor::IOProvider>(m, "IOProvider")
    .def(py::init<std::string, std::size_t>(), py::arg("device"), py::arg("en_idx"))
    .def("set_id", &Adaptor::IOProvider::set_id)
    .def("get_id", &Adaptor::IOProvider::get_id);
}

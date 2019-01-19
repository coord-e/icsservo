cmake_minimum_required(VERSION 2.7)

pybind11_add_module(icsservo SHARED binding.cpp)
target_link_libraries(icsservo servo_lib ioprovider_lib)

install(TARGETS icsservo COMPONENT python DESTINATION lib)

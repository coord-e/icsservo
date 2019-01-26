cmake_minimum_required(VERSION 2.7)

pybind11_add_module(icsservo SHARED binding.cpp)
target_link_libraries(icsservo PRIVATE icsservo_lib)
ics_set_compile_options(icsservo)

install(TARGETS icsservo COMPONENT python DESTINATION ${LIB_INSTALL_DIR})

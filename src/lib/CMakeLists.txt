cmake_minimum_required(VERSION 2.7)

add_library(icsservo_lib STATIC servo.cpp ioprovider.cpp)
set_target_properties(icsservo_lib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(icsservo_lib PROPERTIES OUTPUT_NAME "icsservo")
ics_set_compile_options(icsservo_lib)

install(TARGETS icsservo_lib ARCHIVE DESTINATION ${LIB_INSTALL_DIR})

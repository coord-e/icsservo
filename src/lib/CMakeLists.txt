cmake_minimum_required(VERSION 2.7)

add_library(ioprovider_lib STATIC ioprovider.cpp)
set_target_properties(ioprovider_lib PROPERTIES OUTPUT_NAME "icsservo-ioprovider")
set_target_properties(ioprovider_lib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

add_library(servo_lib STATIC servo.cpp)
set_target_properties(servo_lib PROPERTIES OUTPUT_NAME "icsservo-servo")
set_target_properties(servo_lib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(servo_lib ioprovider_lib)

add_library(icsservo_lib SHARED)
set_target_properties(icsservo_lib PROPERTIES OUTPUT_NAME "icsservo")
target_link_libraries(icsservo_lib servo_lib)

install(TARGETS ioprovider_lib servo_lib ARCHIVE DESTINATION lib)
install(TARGETS icsservo_lib LIBRARY DESTINATION lib)

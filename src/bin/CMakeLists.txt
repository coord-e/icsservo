cmake_minimum_required(VERSION 2.7)

add_executable(servo_bin servo.cpp)
set_target_properties(servo_bin PROPERTIES OUTPUT_NAME "servo")
target_link_libraries(servo_bin icsservo_lib)

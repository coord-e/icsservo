cmake_minimum_required(VERSION 2.7)

add_executable(servo_bin servo.cpp)
set_target_properties(servo_bin PROPERTIES OUTPUT_NAME "icsservo-servo")
target_link_libraries(servo_bin icsservo_lib)

add_executable(get_id_bin get_id.cpp)
set_target_properties(get_id_bin PROPERTIES OUTPUT_NAME "icsservo-get-id")
target_link_libraries(get_id_bin icsservo_lib)

add_executable(set_id_bin set_id.cpp)
set_target_properties(set_id_bin PROPERTIES OUTPUT_NAME "icsservo-set-id")
target_link_libraries(set_id_bin icsservo_lib)

install(TARGETS servo_bin get_id_bin set_id_bin RUNTIME DESTINATION bin)

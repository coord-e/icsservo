cmake_minimum_required(VERSION 2.7)

add_library(ioprovider_lib STATIC ioprovider.cpp register.cpp)
set_target_properties(ioprovider_lib PROPERTIES OUTPUT_NAME "ioprovider")
set_target_properties(ioprovider_lib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

add_library(icsservo_lib STATIC servo.cpp)
set_target_properties(icsservo_lib PROPERTIES OUTPUT_NAME "icsservo")
set_target_properties(icsservo_lib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(icsservo_lib pca9685_lib)


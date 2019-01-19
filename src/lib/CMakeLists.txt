cmake_minimum_required(VERSION 2.7)

add_library(ioprovider_lib STATIC ioprovider.cpp)
set_target_properties(ioprovider_lib PROPERTIES OUTPUT_NAME "ioprovider")
set_target_properties(ioprovider_lib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

add_library(icsservo_lib STATIC servo.cpp)
set_target_properties(icsservo_lib PROPERTIES OUTPUT_NAME "icsservo")
set_target_properties(icsservo_lib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(icsservo_lib ioprovider_lib)

install(TARGETS ioprovider_lib icsservo_lib ARCHIVE DESTINATION lib)

cmake_minimum_required(VERSION 2.7)

add_library(icsservo_lib STATIC servo.cpp ioprovider.cpp)
set_target_properties(icsservo_lib PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(icsservo_lib PROPERTIES OUTPUT_NAME "icsservo")

servoarray_add_driver(ics_driver NAME ics driver.cpp)
target_link_libraries(ics_driver icsservo)

install(TARGETS icsservo_lib ARCHIVE DESTINATION lib)

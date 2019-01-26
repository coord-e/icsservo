servoarray_add_driver(ics_driver NAME ics ics.cpp)
target_link_libraries(ics_driver icsservo_lib)
ics_set_compile_options(ics_driver)

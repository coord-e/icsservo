cmake_minimum_required(VERSION 3.7)
cmake_policy(SET CMP0048 NEW)

project(icsservo VERSION 0.1 LANGUAGES CXX)

if(NOT CONFIG)
  set(CONFIG Release)
endif()

option(BUILD_SERVOARRAY_DRIVER "Build a driver for servoarray" ON)
option(BUILD_PYTHON_MODULE "Build a python binding" ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PYBIND11_CPP_STANDARD -std=c++14)

include(cmake/compile-options.cmake)

include_directories(include/)

add_subdirectory(include)

add_subdirectory(lib)
add_subdirectory(bin)

if(${BUILD_SERVOARRAY_DRIVER})
  find_package(servoarray 0.1 REQUIRED)
  include_directories(${SERVOARRAY_INCLUDE_DIR})

  add_subdirectory(driver)
endif()

if(${BUILD_PYTHON_MODULE})
  add_subdirectory(pybind11)
  # pybind11 headers produce so many errors with strict compiler options
  include_directories(SYSTEM ${PYBIND11_INCLUDE_DIR})

  add_subdirectory(python)
endif()

include(cmake/cpack.cmake)

cmake_minimum_required(VERSION 2.8.4)
project(inb4)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    "inb4.h"
    "inb4.cpp"
)

add_executable(inb4 ${SOURCE_FILES})
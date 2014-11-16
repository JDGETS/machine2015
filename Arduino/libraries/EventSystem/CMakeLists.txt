cmake_minimum_required(VERSION 2.8.4)
project(EventSystem)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    "EventDispatcher.h"
    "EventDispatcher.cpp"
    "EventQueue.h"
    "EventQueue.cpp"
    "Events.h"
)

add_executable(EventSystem ${SOURCE_FILES})
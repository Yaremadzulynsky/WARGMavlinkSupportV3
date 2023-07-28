cmake_minimum_required(VERSION 3.25)
project(WARGMavlinkSupport)

set(CMAKE_CXX_STANDARD 17)

add_executable(WARGMavlinkSupport main.cpp MavlinkDecoder.h MavlinkDecoder.cpp BasicMavlinkEncoder.h BasicMavlinkEncoder.cpp)

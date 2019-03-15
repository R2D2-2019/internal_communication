cmake_minimum_required(VERSION 3.2)
project(R2D2)

set(CMAKE_CXX_STANDARD 17)
add_executable(R2D2 ./code/main.cpp)
add_executable(R2D2-Test ./test/main.cpp)
add_definitions(-D__SAM3X8E__)
include_directories(./code/headers
        ../../libraries/hwlib/library
        ../../programs/bmptk/targets/cortex/atmel
)

target_include_directories(R2D2-Test PUBLIC ../../libraries/Catch2/single_include/catch2)
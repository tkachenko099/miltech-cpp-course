cmake_minimum_required(VERSION 3.16)

project(ugv_odometry)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(ugv_odometry src/main.cpp)
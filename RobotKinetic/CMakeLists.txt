cmake_minimum_required(VERSION 3.26)
project(RobotKinetic)

set(CMAKE_CXX_STANDARD 17)

find_package(eigen3 REQUIRED)

include_directories("/usr/local/include/eigen3")

add_executable(RobotKinetic main.cpp
        kinetic.cpp
)

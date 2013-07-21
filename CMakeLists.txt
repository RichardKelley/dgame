cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME dgame)
include(cmake/pods.cmake)

add_definitions(-std=c++0x)

# create an executable, and make it public
add_executable(dgame src/t1.cpp)

pods_install_executables(dgame)

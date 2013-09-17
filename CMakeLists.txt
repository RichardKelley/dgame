cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME dgame)
include(cmake/pods.cmake)

add_definitions(-std=c++0x)

# create an executable, and make it public
add_executable(${POD_NAME} src/t1.cpp)

set(REQUIRED_PACKAGES smpl lcm bot2-lcmgl-client bot2-core bot2-vis bot2-lcmgl-renderer)
pods_use_pkg_config_packages(${POD_NAME} ${REQUIRED_PACKAGES})

pods_install_executables(${POD_NAME})

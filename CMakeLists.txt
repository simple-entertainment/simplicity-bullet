cmake_minimum_required(VERSION 3.2)

include(../simplicity/simplicity-common.cmake)

# Project
project(simplicity-bullet)

# Paths
file(GLOB_RECURSE SRC_FILES src/bullet/c++/*.cpp src/bullet/c++/*.h src/main/c++/*.cpp src/main/c++/*.h)
include_directories(../simplicity/src/main/c++)
include_directories(src/bullet/c++)
include_directories(src/bullet/c++/bullet)
include_directories(src/main/c++)

# Output
add_library(simplicity-bullet STATIC ${SRC_FILES})

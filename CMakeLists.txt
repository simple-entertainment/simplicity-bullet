cmake_minimum_required(VERSION 3.2)

include(../simplicity/simplicity-common.cmake)

# Project
#########################
project(simplicity-bullet)

# Source
#########################
file(GLOB_RECURSE SRC_FILES src/bullet/c++/*.cpp src/bullet/c++/*.h src/main/c++/*.cpp src/main/c++/*.h)

# Target
#########################
add_library(simplicity-bullet STATIC ${SRC_FILES})
target_include_directories(simplicity-bullet PUBLIC src/main/c++)

# Target Dependencies
#########################

# Bullet
target_include_directories(simplicity-bullet PUBLIC src/bullet/c++)
target_include_directories(simplicity-bullet PUBLIC src/bullet/c++/bullet)

# Simplicity
target_link_libraries(simplicity-bullet simplicity)

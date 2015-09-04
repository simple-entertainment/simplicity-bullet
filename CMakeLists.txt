cmake_minimum_required(VERSION 3.2)

include(../simplicity/simplicity-common.cmake)

# Project
#########################
project(simplicity-bullet)

# Project Dependencies
#########################

# Bullet
add_subdirectory(lib/bullet3)

# Source
#########################
file(GLOB_RECURSE SRC_FILES src/main/c++/*.cpp src/main/c++/*.h)

# Target
#########################
add_library(simplicity-bullet ${SIMPLE_LINK_TYPE} ${SRC_FILES})
target_include_directories(simplicity-bullet PUBLIC src/main/c++)

# Target Dependencies
#########################

# Bullet
target_include_directories(simplicity-bullet PUBLIC lib/bullet3/src)
target_link_libraries(simplicity-bullet BulletDynamics)
target_link_libraries(simplicity-bullet BulletCollision)
target_link_libraries(simplicity-bullet LinearMath)

# Simplicity
target_link_libraries(simplicity-bullet simplicity)

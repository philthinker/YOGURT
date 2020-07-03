cmake_minimum_required(VERSION 3.4)

# project name
project(FleshWaxberry)

# set standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find package
#find_package(franka REQUIRED)

# add the LACTIC library
add_subdirectory(LACTIC)

# add executable files to generate
add_executable(CMAKE_test cmake_test.cpp)

target_link_libraries(CMAKE_test PUBLIC LACTIC)
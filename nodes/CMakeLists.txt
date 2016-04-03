cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(bare_earth)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (morpho morpho.cpp)
target_link_libraries (morpho ${PCL_LIBRARIES})

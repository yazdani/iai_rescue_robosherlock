cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(global_hypothesis_verification)

#Pcl
find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (hypothezer hypothezer.cpp)
target_link_libraries (hypothezer ${PCL_LIBRARIES})
 
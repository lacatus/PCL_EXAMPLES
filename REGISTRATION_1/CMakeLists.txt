cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(REGISTRATION_1)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (registration_5 registration_5.cpp)
target_link_libraries (registration_5 ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(HOLA_KINECT_2)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (openni_grabber openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

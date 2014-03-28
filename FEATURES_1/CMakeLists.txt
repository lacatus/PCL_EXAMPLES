cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(FEATURES_1)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (features_register_1 features_register_1.cpp)
target_link_libraries (features_register_1 ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(SACA_FOTO)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (saca_foto_pcd saca_foto_pcd.cpp)
target_link_libraries (saca_foto_pcd ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PASS_T_PCD)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pass_t_pcd pass_t_pcd.cpp)
target_link_libraries (pass_t_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS})

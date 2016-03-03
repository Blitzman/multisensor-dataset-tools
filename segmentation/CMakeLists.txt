cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_segmentation)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_segmentation pcd_segmentation.cpp)
target_link_libraries (pcd_segmentation ${PCL_LIBRARIES})
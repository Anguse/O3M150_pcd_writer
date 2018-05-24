cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(res/Channels)
include_directories(res/src)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(res/Channels)
link_directories(res/src)

add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

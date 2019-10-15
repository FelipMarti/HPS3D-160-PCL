cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(read_visualise_cloud)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (read_visualise_cloud read_visualise_cloud.cpp)
target_link_libraries (read_visualise_cloud ${PCL_LIBRARIES})


cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(HPS3D_PCL_SAVE_HEAPS_CLOUDS)

find_package(PCL 1.2 REQUIRED COMPONENTS)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}  ${CMAKE_BINARY_DIR}/lib)
add_definitions(${PCL_DEFINITIONS})

add_executable(hps3d-pcl-save-heaps-clouds hps3d-pcl-save-heaps-clouds.cpp)
target_link_libraries(hps3d-pcl-save-heaps-clouds ${PCL_LIBRARIES} hps3d64)


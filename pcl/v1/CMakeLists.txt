project(PCL_LoadPlyFile)
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
aux_source_directory(. SRC_LIST)

set(PCL_DIR "/home/julian/libraries/pcl-pcl-trunk/PCLConfig.cmake")

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization sample_consensus segmentation)
find_package(OpenCV REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(PCL_loadfile pcl_test3.cpp)
target_link_libraries(PCL_loadfile ${PCL_LIBRARIES} ${OpenCV_LIBS})



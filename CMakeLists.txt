cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cloud_viewer)
variable_watch(VTK_DIR)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(loader_includes
    src/kitti_data_loader.h
    src/point_xyzr.h)

set(loader_sources
    src/kitti_data_loader.cpp
    )

add_library(kitti_loader ${loader_includes} ${loader_sources})

set(sources
    src/cloud_viewer.cpp
    )
add_executable (cloud_viewer ${includes} ${sources})
target_link_libraries (cloud_viewer ${PCL_LIBRARIES} kitti_loader)

set(sources_ec
src/crfground.h
src/crfground.cpp
src/eucledian_clustering.cpp
)
add_executable (eucledian_clustering ${sources_ec})
target_link_libraries (eucledian_clustering ${PCL_LIBRARIES} kitti_loader)

add_executable (ground_estimation
    src/ground_estimation.cpp
    src/ground_estimation.h)
target_link_libraries (ground_estimation ${PCL_LIBRARIES} kitti_loader)


cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

set(sources
    src/cloud_viewer.cpp
    )
add_executable (cloud_viewer ${includes} ${sources})
target_link_libraries (cloud_viewer ${PCL_LIBRARIES} KITTILoader::DataLoader ${Boost_LIBRARIES})

set(sources_ec
src/crfground.h
src/crfground.cpp
src/eucledian_clustering.cpp
)
add_executable (eucledian_clustering ${sources_ec})
target_link_libraries (eucledian_clustering ${PCL_LIBRARIES} KITTILoader::DataLoader)

add_executable (ground_estimation
    src/ground_estimation.cpp
    src/ground_estimation.h)

target_link_libraries (ground_estimation ${PCL_LIBRARIES} PCLThreadedViewer::Viewer KITTILoader::DataLoader ${Boost_LIBRARIES})

add_subdirectory(KITTI_Loader)

add_subdirectory(PCL_Viewer)

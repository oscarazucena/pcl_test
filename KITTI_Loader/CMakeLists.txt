cmake_minimum_required(VERSION 3.5)

project(KITTI_DATA_LOADER VERSION 0.0.1 LANGUAGES CXX)

find_package(PCL 1.2 REQUIRED COMPONENTS common)

add_library(kitti_loader
    src/kitti_data_loader.cpp
    src/kitti_time_stamp_data.cpp
    include/kitti_data_loader.h
    include/kitti_time_stamp_data.h
    include/point_xyzr.h
    )

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(KITTILoader::DataLoader ALIAS kitti_loader)

#Set target properties
target_include_directories(kitti_loader
    PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

target_compile_features(kitti_loader PRIVATE cxx_auto_type)

target_link_libraries(kitti_loader
    PUBLIC
    ${PCL_COMMON_LIBRARIES} KITTILoader::DataLoader
    )


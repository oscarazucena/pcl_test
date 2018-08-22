cmake_minimum_required(VERSION 3.5)

project(PCL_Threaded_Viewer VERSION 0.0.1 LANGUAGES CXX)

find_package(PCL 1.2 REQUIRED COMPONENTS)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(pcl_threaded_viewer
    src/pcl_threaded_viewer.cpp
    src/pcl_text_info.cpp
    include/pcl_threaded_viewer.h
    include/pcl_text_info.h
    )

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(PCLThreadedViewer::Viewer ALIAS pcl_threaded_viewer)

#Set target properties
target_include_directories(pcl_threaded_viewer
    PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${PCL_INCLUDE_DIRS}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

target_compile_features(pcl_threaded_viewer PRIVATE cxx_auto_type cxx_template_template_parameters)

target_link_libraries(pcl_threaded_viewer
    PUBLIC
    ${PCL_COMMON_LIBRARIES}
    PCLThreadedViewer::Viewer
    KITTILoader::DataLoader
    )


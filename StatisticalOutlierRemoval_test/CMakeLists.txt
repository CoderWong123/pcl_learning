cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(statistical_removal)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (statistical_outlier_removal_test src/statistical_outlier_removal_test.cpp)
target_link_libraries (statistical_outlier_removal_test ${PCL_LIBRARIES})

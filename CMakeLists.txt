cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(rgbd_data_analysis)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pipeline pipeline.cpp)
target_link_libraries(pipeline ${PCL_LIBRARIES})

add_executable(pipeline_file pipeline_file.cpp)
target_link_libraries(pipeline_file ${PCL_LIBRARIES})

add_executable(ism_train ism_train.cpp)
target_link_libraries(ism_train ${PCL_LIBRARIES})

add_executable(ism_test ism_test.cpp)
target_link_libraries(ism_test ${PCL_LIBRARIES})

add_executable(viewer viewer.cpp)
target_link_libraries(viewer ${PCL_LIBRARIES})

add_executable(ism_pipeline ism_pipeline.cpp)
target_link_libraries(ism_pipeline ${PCL_LIBRARIES})

add_executable(ism_pipeline_file ism_pipeline_file.cpp)
target_link_libraries(ism_pipeline_file ${PCL_LIBRARIES})

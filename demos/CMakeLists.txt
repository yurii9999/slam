cmake_minimum_required(VERSION 2.8)

project(demos)
# in ./additional stored some classes that can be useful to write demos
# additional/sequence_parameters.h -- read&store parameters of kitti sequence
# additional/egomotion_parameters.h -- read&store parameters of egomotion_estimation
# additional/segmentation_parameters.h -- parameters of segmentation
# additional/fabric.h -- create objects of tracker, egomotion_estimation, segmentation according parameters

include_directories(${CMAKE_SOURCE_DIR}/external/include)
#add_executable(demo01 "demo0.cpp")
#target_link_libraries(demo01 slam)

add_executable(demo1 "demo1.cpp")
target_link_libraries(demo1 slam)

add_executable(demo2 "demo2.cpp" "additional/sequence_parameters.h" "additional/egomotion_parameters.h" "additional/factory.h" "additional/bucketing_parameters.h")
target_link_libraries(demo2 slam)

add_executable(demo3 "demo3.cpp" "additional/sequence_parameters.h" "additional/segmentation_parameters.h")
target_link_libraries(demo3 slam)

add_executable(trackertest "trackertest.cpp")
target_link_libraries(trackertest slam)

add_executable(sceneflow "sceneflow.cpp")
target_link_libraries(sceneflow slam)

#add_executable(relativePoseDemo "relativeposedemo.cpp")
#target_link_libraries(relativePoseDemo slam)

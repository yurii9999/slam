cmake_minimum_required(VERSION 2.8)

project(slam)

FILE(GLOB SRC_FILES "src/*.cpp")
FILE(GLOB INCLUDE_FILES "include/*.h")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

add_library(${PROJECT_NAME} ${SRC_FILES} ${INCLUDE_FILES})

find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBS})

include_directories(${CMAKE_SOURCE_DIR}/external/include)
#opengv
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/external/lib/libopengv.a)
#libviso2
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/external/lib/libviso2.a)
##delaunay-triangulation
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/external/lib/libdelaunay-triangulation.so)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)




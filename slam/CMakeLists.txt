cmake_minimum_required(VERSION 2.8)

project(slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FILE(GLOB SRC_FILES "src/*.cpp")

add_library(${PROJECT_NAME} ${SRC_FILES})

find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBS})

target_link_libraries(${PROJECT_NAME} sophus)
target_link_libraries(${PROJECT_NAME} viso2)

target_link_libraries(${PROJECT_NAME} opengv)

add_definitions(-march=native)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)

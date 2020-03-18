cmake_minimum_required(VERSION 2.8)

project(demos)
add_executable(demo1 "demo1.cpp")

target_link_libraries(demo1 slam)

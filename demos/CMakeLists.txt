cmake_minimum_required(VERSION 2.8)

project(demos)
add_executable(demo1 "demo1.cpp")

target_link_libraries(demo1 slam)


add_executable(demo2 "demo2.cpp")
target_link_libraries(demo2 slam)

#add_executable(relativePoseDemo "relativeposedemo.cpp")
#target_link_libraries(relativePoseDemo slam)

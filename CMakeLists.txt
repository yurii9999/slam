cmake_minimum_required(VERSION 2.8)

project(slam)
add_executable(${PROJECT_NAME} "main.cpp" Tracker.cpp Tracker.h ScenePoint.cpp ScenePoint.h Projection.cpp Projection.h Camera.cpp Camera.h Scene.cpp Scene.h RegularCamera.cpp RegularCamera.h FrameKeeper.cpp FrameKeeper.h RegularFrame.cpp RegularFrame.h FancyDrawer.cpp FancyDrawer.h)

find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


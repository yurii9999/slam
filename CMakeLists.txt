project(project)

add_definitions(-march=native)

add_subdirectory(slam)
add_subdirectory(demos)

add_subdirectory(external/Sophus)
add_subdirectory(external/libviso2)
add_subdirectory(external/opengv)
add_subdirectory(external/cxxopts)

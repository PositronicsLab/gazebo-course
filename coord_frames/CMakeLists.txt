cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(coord_frame_planar SHARED coord_frame_planar.cpp)
add_library(coord_frame_planar2 SHARED coord_frame_planar2.cpp)
add_library(coord_frame_spatial SHARED coord_frame_spatial.cpp)
target_link_libraries(coord_frame_spatial ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


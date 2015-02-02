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

add_library(PD_hold SHARED PD_hold.cpp)
add_library(PID_hold SHARED PID_hold.cpp)
add_library(track_sinusoidal SHARED track_sinusoidal.cpp)
target_link_libraries(PD_hold ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(PID_hold ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(track_sinusoidal ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#add_library(ur10_fk SHARED ur10_fk.cc)
#target_link_libraries(ur10_fk ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


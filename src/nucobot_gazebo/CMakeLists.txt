cmake_minimum_required(VERSION 2.8.3)
project(nucobot_gazebo)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs std_srvs geometry_msgs nav_msgs tf)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})



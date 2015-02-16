cmake_minimum_required(VERSION 2.8.3)
project(odometry_driver_wheel)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation message_runtime
                                        sensor_msgs std_msgs roscpp tf)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(odometry_wheel_driver odometry_wheel_driver.cpp)
target_link_libraries(odometry_wheel_driver ${catkin_LIBRARIES})



cmake_minimum_required(VERSION 2.8.3)
project(odometry_driver)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation message_runtime
                                        sensor_msgs std_msgs roscpp rospy tf)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(odometry_hw2ros_driver odometry_hw2ros_driver.cpp)
target_link_libraries(odometry_hw2ros_driver ${catkin_LIBRARIES})



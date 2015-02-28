cmake_minimum_required(VERSION 2.8.3)
project(imu_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(imu_driver_node imu_driver_node.cpp)

target_link_libraries(imu_driver_node ${catkin_LIBRARIES})


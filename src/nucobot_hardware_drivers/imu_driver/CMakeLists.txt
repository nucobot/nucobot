cmake_minimum_required(VERSION 2.8.3)
project(imu_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_driver_node src/imu_driver_node.cpp)

 target_link_libraries(imu_driver_node
   ${catkin_LIBRARIES}
 )


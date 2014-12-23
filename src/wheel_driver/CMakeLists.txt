cmake_minimum_required(VERSION 2.8.3)
project(wheel_driver)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport geometry_msgs
                                        message_generation
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        roscpp
            )

add_message_files(
  FILES
  WheelState.msg
)

generate_messages(   
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
) 

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs geometry_msgs)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(wheel_driver wheel_driver.cpp)
target_link_libraries(wheel_driver ${catkin_LIBRARIES})

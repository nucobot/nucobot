cmake_minimum_required(VERSION 2.8.3)
project(camera_driver)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport geometry_msgs
                                        sensor_msgs std_msgs roscpp tf)
find_package(OpenCV 3.0 REQUIRED)


catkin_package()

include_directories(${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS})

add_executable(camera_driver camera_driver.cpp)
target_link_libraries(camera_driver ${catkin_LIBRARIES} 
                                    ${OpenCV_LIBS})



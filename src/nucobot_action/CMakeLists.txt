cmake_minimum_required(VERSION 2.8.3)
project(nucobot_action)


set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS 
                actionlib
                actionlib_msgs
                message_generation
                roscpp
                rospy
                std_msgs
				smach
				smach_ros)

find_package(Boost REQUIRED COMPONENTS system)


FILE(GLOB ACTION_SRCS RELATIVE ${CMAKE_CURRENT_LIST_DIR}/action action/*.action)
add_action_files(
  DIRECTORY action
  FILES ${ACTION_SRCS}
)  


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)


catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)


###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS}
            		${BOOST_INCLUDE_DIRS})


set(SERVER_SOURCES action_server.cpp)
add_executable(nucobot_action_server ${SERVER_SOURCES})

target_link_libraries(nucobot_action_server ${catkin_LIBRARIES})

add_dependencies(nucobot_action_server ${PROJECT_NAME}_gencpp)




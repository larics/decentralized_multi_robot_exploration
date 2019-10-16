cmake_minimum_required(VERSION 2.8.3)
project(frontier_exploration)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
  cartographer_ros
)

find_package(cartographer REQUIRED)

add_message_files(
  FILES
  PointArray.msg
  FrontierTF.msg
  InfoList.msg
  RobotPosGoal.msg
 
)


add_service_files(FILES
  RobotService.srv

)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  cartographer_ros_msgs
)

catkin_package( 
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${cartographer_INCLUDE_DIRS})


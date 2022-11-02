cmake_minimum_required(VERSION 2.8.3)
project(robot_navigation)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf
    message_generation
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  std_msgs
  message_generation
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy tf2 tf2_ros message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tag_follow src/tag_following.cpp)
target_link_libraries(tag_follow ${catkin_LIBRARIES})

add_executable(path_track src/path_tracking.cpp)
target_link_libraries(path_track ${catkin_LIBRARIES})

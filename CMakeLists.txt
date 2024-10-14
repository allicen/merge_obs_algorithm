cmake_minimum_required(VERSION 3.0.2)
project(merge_obs_algorithm)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_srvs
  sensor_msgs
  moveit_core 
  moveit_msgs
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools
  std_msgs
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/octomap.cpp
  src/generate_point_cloud.cpp
  examples/example.cpp
)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}  
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
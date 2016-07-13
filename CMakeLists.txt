cmake_minimum_required(VERSION 2.8.3)
project(baxter_work_space)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES baxter_work_space
#  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

# EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS system )


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(baxter_work_space src/baxter_work_space.cpp)
target_link_libraries(baxter_work_space ${catkin_LIBRARIES})

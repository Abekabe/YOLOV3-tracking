cmake_minimum_required(VERSION 2.8.3)

project(object_tracking)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  darknet_ros_msgs
  cv_bridge
  image_transport
)

find_package(PCL 1.2 REQUIRED)

find_package( OpenCV REQUIRED )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (tracking src/YoloTracker.cpp  src/YoloTracker_node.cpp src/Hungarian.cpp)
target_link_libraries (tracking ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})



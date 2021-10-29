cmake_minimum_required(VERSION 3.0.2)
project(bender_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  image_geometry
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED COMPONENTS
  opencv_ximgproc
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(bender_perception_lib
  src/vision.cpp 
  src/lidar.cpp
  src/bitarray_to_laserscan.cpp
)
add_dependencies(bender_perception_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bender_perception_lib
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}  
)

add_executable(${PROJECT_NAME} src/bender_perception.cpp)
target_link_libraries(${PROJECT_NAME} 
  bender_perception_lib
  ${catkin_LIBRARIES} 
)

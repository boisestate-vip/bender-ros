cmake_minimum_required(VERSION 3.0.2)
project(bender_navsensors)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(bender_navsensors_lib
  src/bender_navsensors.cpp
)

add_dependencies(bender_navsensors_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(bender_navsensors_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME} src/bender_navsensors.cpp)

target_link_libraries(${PROJECT_NAME}
  bender_navsensors_lib
  ${catkin_LIBRARIES}
)


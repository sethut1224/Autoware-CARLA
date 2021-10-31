cmake_minimum_required(VERSION 2.8.3)
project(op_direct_controller)

find_package(autoware_build_flags REQUIRED)

find_package(OpenCV REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  autoware_msgs
  autoware_can_msgs
  geometry_msgs  
  op_planner  
  op_utility
  op_ros_helpers
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_msgs
    autoware_can_msgs
    geometry_msgs    
    op_planner   
    op_utility
    op_ros_helpers
    roscpp
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)
add_executable(
  op_direct_controller
  nodes/op_direct_controller.cpp
  nodes/op_direct_controller_core.cpp
)
target_link_libraries(
  op_direct_controller
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(
  op_direct_controller
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS op_direct_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)

cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME tracking)

project(${PROJECT_NAME})

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS module/include
#  LIBRARIES curb_detection
#  CATKIN_DEPENDS pcl_ros roscpp rospy
#  DEPENDS system_lib
)

aux_source_directory(module/src SOURCE_SRCS)
aux_source_directory(module/src/matching MATCHING_SOURCE_SRCS)
aux_source_directory(module/src/thirdPart THIRD_SOURCE_SRCS)
aux_source_directory(apps APP_SRCS)

link_directories(
  module/lib
  ${catkin_LIB_DIRS}
)

include_directories(
  module/include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet
  ${MATCHING_SOURCE_SRCS}
  ${THIRD_SOURCE_SRCS}
  ${SOURCE_SRCS}
  ${APP_SRCS}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)

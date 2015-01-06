cmake_minimum_required(VERSION 2.8.3)
project(zbar_ros)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge tf nodelet)

find_package(Zbar REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zbar_ros
  CATKIN_DEPENDS nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

add_library(zbar_ros src/zbar_ros_base.cpp src/barcode_reader_nodelet)
target_link_libraries(zbar_ros ${catkin_LIBRARIES} ${ZBAR_LIBRARIES})

add_executable(barcode_reader src/barcode_reader_node.cpp)
target_link_libraries(barcode_reader zbar_ros ${catkin_LIBRARIES})

# add_dependencies(test husky_msgs_gencpp)
cmake_minimum_required(VERSION 2.8.3)
project(zbar_ros)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge tf nodelet roslint)

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

add_library(zbar_ros src/zbar_ros_base.cpp src/barcode_reader_nodelet.cpp)
target_link_libraries(zbar_ros zbar ${catkin_LIBRARIES})

add_executable(barcode_reader src/barcode_reader_node.cpp)
target_link_libraries(barcode_reader zbar_ros ${catkin_LIBRARIES})

roslint_cpp()

install(TARGETS barcode_reader zbar_ros
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

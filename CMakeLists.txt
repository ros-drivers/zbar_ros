cmake_minimum_required(VERSION 2.8.3)
project(zbar_ros)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge nodelet roslint)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES barcode_reader_nodelet
  CATKIN_DEPENDS nodelet cv_bridge roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(barcode_reader_nodelet src/barcode_reader_nodelet.cpp)
target_link_libraries(barcode_reader_nodelet zbar ${catkin_LIBRARIES})

add_executable(barcode_reader_node src/barcode_reader_node.cpp)
target_link_libraries(barcode_reader_node ${catkin_LIBRARIES})

roslint_cpp()

install(TARGETS barcode_reader_node barcode_reader_nodelet
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

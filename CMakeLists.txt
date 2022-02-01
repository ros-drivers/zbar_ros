cmake_minimum_required(VERSION 3.5)
project(zbar_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Include header files
include_directories(
  include)

# Build library
add_library(barcode_reader_node src/barcode_reader_node.cpp)
ament_target_dependencies(barcode_reader_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge)
target_link_libraries(barcode_reader_node zbar)

# Build executable
add_executable(barcode_reader src/barcode_reader_main.cpp)
target_link_libraries(barcode_reader barcode_reader_node)

# Install
install(TARGETS
  barcode_reader
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/)
install(
  TARGETS barcode_reader_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright check, because the copyright used is not recognised by ament_copyright
  set(ament_cmake_copyright_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()

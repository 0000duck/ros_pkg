cmake_minimum_required(VERSION 3.5)
project(stewart_hmi_pkg)

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
find_package(stewart_interfaces_pkg REQUIRED)

# Add executables
add_executable(hmi src/hmi_node.cpp src/oscillator.cpp)
ament_target_dependencies(hmi rclcpp stewart_interfaces_pkg)

# Include the directories where header files in found
include_directories(include/stewart_hmi_pkg)

install(TARGETS
  hmi
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()



cmake_minimum_required(VERSION 3.5)
project(stewart_hmi_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

IF (NOT DEFINED BUILD_VERSION)
	SET(BUILD_VERSION "not set")
ENDIF()
ADD_DEFINITIONS(-DBUILD_VERSION="${BUILD_VERSION}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stewart_interfaces_pkg REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)

if(rclcpp_VERSION_MAJOR GREATER_EQUAL 2)
    message("Foxy or newer rclcpp detected")
    add_compile_definitions(POST_FOXY)
endif(rclcpp_VERSION_MAJOR GREATER_EQUAL 2)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include the directories where header files in found
include_directories(
  include/stewart_hmi_pkg
	${rclcpp_INCLUDE_DIRS}
	${Qt5Core_INCLUDE_DIRS}
	${Qt5Quick_INCLUDE_DIRS}
)

file(GLOB SOURCE_FILES
	"src/*.cpp"
	"src/*.h"
  "include/stewart_hmi_pkg/*.cpp"
)

# Add executables
add_executable(hmi_node 
  ${SOURCE_FILES}
  "src/qml.qrc"
)

ament_target_dependencies(hmi_node
  rclcpp
  stewart_interfaces_pkg
  rmw_implementation
)

target_link_libraries(hmi_node
  Qt5::Core
  Qt5::Quick
)

install(TARGETS
  hmi_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()



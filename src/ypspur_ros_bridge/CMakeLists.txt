cmake_minimum_required(VERSION 2.8.3)
project(ypspur_ros_bridge)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy nav_msgs geometry_msgs tf)

find_library(YPSPUR_LIBRARY NAMES ypspur libypspur)
if(NOT YPSPUR_LIBRARY)
	message(FATAL_ERROR "YP-Spur should be installed. https://openspur.org/redmine/projects/ypspur/")
endif()

## Declare a catkin package
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs tf
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Build rosImasenWheelchair
include_directories(${catkin_INCLUDE_DIRS})

add_executable(RosYPSpur src/RosYPSpur.cpp)
target_link_libraries(RosYPSpur ypspur ${catkin_LIBRARIES})

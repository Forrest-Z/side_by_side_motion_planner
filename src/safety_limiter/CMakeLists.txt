cmake_minimum_required(VERSION 2.8.3)
project(safety_limiter)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy nav_msgs std_msgs geometry_msgs tf sensor_msgs laser_geometry message_generation
  pcl_conversions
  pcl_ros
  tf_conversions
  )

add_message_files(
	FILES Status.msg
)
generate_messages(
	DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(
  DEPENDS roscpp nav_msgs geometry_msgs tf sensor_msgs laser_geometry message_runtime
  pcl_conversions pcl_ros
  tf_conversions
    )
  
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
## Build 
include_directories(${catkin_INCLUDE_DIRS})

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

add_executable(safety_limiter src/safety_limiter.cpp)
target_link_libraries(safety_limiter ${catkin_LIBRARIES})
add_dependencies(safety_limiter safety_limiter_generate_messages_cpp)


add_executable( footprint_2_cloud src/footprint_2_cloud.cpp)
target_link_libraries(footprint_2_cloud ${catkin_LIBRARIES})
#add_dependencies(footprint_2_cloud)


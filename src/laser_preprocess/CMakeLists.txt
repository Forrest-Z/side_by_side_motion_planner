cmake_minimum_required(VERSION 2.8.3)
project(laser_preprocess)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rospy nav_msgs geometry_msgs tf sensor_msgs laser_assembler laser_geometry eigen_conversions pcl_ros)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

## Declare a catkin package
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs tf sensor_msgs eigen eigen_conversions pcl_ros
    CATKIN_DEPENDS laser_assembler laser_geometry
)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_DEFINITIONS(-O3)

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

## Build 
add_executable(uni_density src/uni_density.cpp)
add_executable(uni_density2d src/uni_density2d.cpp)
add_executable(merge_clouds src/merge_clouds.cpp)
add_executable(cut src/cut.cpp)
add_executable(periodic_assemble_scans src/periodic_assemble_scans.cpp)
add_executable(periodic_assemble_cloud2s src/periodic_assemble_cloud2s.cpp)
add_executable(pc2_transformer src/pc2_transformer.cpp)
target_link_libraries(uni_density ${catkin_LIBRARIES})
target_link_libraries(uni_density2d ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(merge_clouds ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cut ${catkin_LIBRARIES})
target_link_libraries(periodic_assemble_scans ${catkin_LIBRARIES})
target_link_libraries(periodic_assemble_cloud2s ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pc2_transformer ${catkin_LIBRARIES})


cmake_minimum_required(VERSION 2.8.3)
project(grid_local_planner)

find_package(catkin REQUIRED
	COMPONENTS base_local_planner cmake_modules costmap_2d 
		nav_core nav_msgs pluginlib pcl_conversions roscpp tf safety_limiter)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    LIBRARIES grid_local_planner
    CATKIN_DEPENDS
        pluginlib
        roscpp
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

CHECK_CXX_COMPILER_FLAG("-ffast-math" COMPILER_SUPPORTS_FASTMATH)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
	if(COMPILER_SUPPORTS_FASTMATH)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
	endif()
endif()

add_library(grid_local_planner src/grid_planner_ros.cpp src/global_planner.cpp)
target_link_libraries(grid_local_planner base_local_planner ${catkin_LIBRARIES})
add_dependencies(grid_local_planner grid_local_planner_gencfg)
add_dependencies(grid_local_planner nav_msgs_gencpp)

install(TARGETS grid_local_planner
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


cmake_minimum_required(VERSION 2.8.3)
project(simple_simulator)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs sensor_msgs tf)

find_package( OpenMP REQUIRED)
if(OPENMP_FOUND)
	message("OPENMP FOUND")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

## Declare a catkin package
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odo_simulator src/odo_simulator.cpp)
target_link_libraries(odo_simulator ${catkin_LIBRARIES})

add_executable(utm_simulator src/utm_simulator.cpp)
target_link_libraries(utm_simulator ${catkin_LIBRARIES})

add_executable(ackermann_simulator src/ackermann_simulator.cpp)
target_link_libraries(ackermann_simulator ${catkin_LIBRARIES})
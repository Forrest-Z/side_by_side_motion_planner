cmake_minimum_required(VERSION 2.8.3)
project(joystick_teleop)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs)

## Declare a catkin package
catkin_package(
    DEPENDS roscpp sensor_msgs geometry_msgs
)

## Build joystick_teleop_node
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(joystick_teleop_node src/joystick_teleop_node.cpp)
target_link_libraries(joystick_teleop_node ${catkin_LIBRARIES})

add_executable(ackermann_joystick_teleop_node src/ackermann_joystick_teleop_node.cpp)
target_link_libraries(ackermann_joystick_teleop_node ${catkin_LIBRARIES})

add_executable(button_cmd_mapper src/button_cmd_mapper.cpp)
target_link_libraries(button_cmd_mapper ${catkin_LIBRARIES})
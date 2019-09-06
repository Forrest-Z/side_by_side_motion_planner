cmake_minimum_required(VERSION 2.8.3)
project(dynamic_object_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(FILES DynamicObject.msg DynamicObjectArray.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(
    DEPENDS message_runtime std_msgs geometry_msgs
)




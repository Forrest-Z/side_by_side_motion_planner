cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation nav_msgs geometry_msgs tf interactive_markers)

add_service_files(FILES ChangePath.srv)
add_message_files(FILES TrajectoryTrackerStatus.msg TrajectoryServerStatus.msg TrajectoryMessage.msg 
TrajectoryPath.msg 
)
                                                                               
generate_messages(DEPENDENCIES std_msgs geometry_msgs )

## Declare a catkin package
catkin_package(
    DEPENDS system_lib roscpp message_runtime std_msgs nav_msgs geometry_msgs tf interactive_markers 
)

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

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(trajectory_tracker src/trajectory_tracker.cpp)
target_link_libraries(trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(trajectory_tracker trajectory_tracker_generate_messages_cpp)


add_executable(trajectory_recorder src/trajectory_recorder.cpp)
target_link_libraries(trajectory_recorder ${catkin_LIBRARIES})
add_dependencies(trajectory_recorder trajectory_tracker_generate_messages_cpp)

add_executable(trajectory_saver src/trajectory_saver.cpp)
target_link_libraries(trajectory_saver ${catkin_LIBRARIES})
add_dependencies(trajectory_saver trajectory_tracker_generate_messages_cpp)

add_executable(trajectory_server src/trajectory_server.cpp src/filter.cpp)
target_link_libraries(trajectory_server ${catkin_LIBRARIES})
add_dependencies(trajectory_server trajectory_tracker_gencpp trajectory_server_generate_messages_cpp)


add_executable(ackermann_trajectory_tracker src/ackermann_trajectory_tracker.cpp)
target_link_libraries(ackermann_trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(ackermann_trajectory_tracker trajectory_tracker_generate_messages_cpp)

add_executable(file_to_path src/file_to_path.cpp)
target_link_libraries( file_to_path ${catkin_LIBRARIES})
add_dependencies(file_to_path trajectory_tracker_generate_messages_cpp)



add_executable(no_velocity_trajectory_tracker src/no_velocity_trajectory_tracker.cpp)
target_link_libraries(no_velocity_trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(no_velocity_trajectory_tracker trajectory_tracker_generate_messages_cpp)




add_executable(w_ackermann_trajectory_tracker src/w_ackermann_trajectory_tracker.cpp)
target_link_libraries(w_ackermann_trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(w_ackermann_trajectory_tracker trajectory_tracker_generate_messages_cpp)


add_executable(fb_ackermann_trajectory_tracker src/fb_ackermann_trajectory_tracker.cpp)
target_link_libraries(fb_ackermann_trajectory_tracker ${catkin_LIBRARIES})
add_dependencies(fb_ackermann_trajectory_tracker trajectory_tracker_generate_messages_cpp)

add_executable(path_with_time src/path_with_time.cpp)
target_link_libraries(path_with_time ${catkin_LIBRARIES})

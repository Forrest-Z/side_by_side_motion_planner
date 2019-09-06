cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_tools)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  message_filters
  velodyne_pointcloud
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

#find_package(PCL REQUIRED) 
#include_directories(${PCL_INCLUDE_DIRS}) 
#add_definitions(${PCL_DEFINITIONS}) 

#find_package(VTK REQUIRED) 
#include( ${VTK_USE_FILE} ) 

catkin_package(
   CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf tf_conversions message_filters
   DEPENDS system_lib octomap
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(octomap REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(pc_transformer src/pc_transformer.cpp)
target_link_libraries(pc_transformer ${catkin_LIBRARIES})

add_executable(plane_model_node src/plane_model_node.cpp)
target_link_libraries(plane_model_node ${catkin_LIBRARIES})

add_executable(refine_ot src/refine_ot.cpp)
target_link_libraries(refine_ot ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

#add_executable(fixed_transformer src/fixed_transformer.cpp)
#target_link_libraries(fixed_transformer ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(pcd_to_octomap src/pcd_to_octomap.cpp)
target_link_libraries(pcd_to_octomap ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

#add_executable(map_ground_removal src/map_ground_removal.cpp)
#target_link_libraries(map_ground_removal ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

#add_executable(map_ground_removal3 src/map_ground_removal3.cpp)
#target_link_libraries(map_ground_removal3 ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  ${OCTOMAP_LIBRARIES}  ${VTK_LIBRARIES})



add_executable(pointcloud_transform src/pointcloud_transform.cpp)
target_link_libraries(pointcloud_transform ${catkin_LIBRARIES})

add_executable(pointcloud_create_global_map src/pointcloud_create_global_map.cpp)
target_link_libraries(pointcloud_create_global_map ${catkin_LIBRARIES})


#add_executable(pointcloud_filter_by_distance src/pointcloud_filter_by_distance.cpp)
#target_link_libraries(pointcloud_filter_by_distance ${catkin_LIBRARIES}    ${PCL_LIBRARIES} )

cmake_minimum_required(VERSION 2.8.3)
project(ur5_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

## Find source installed PCL, comment this if use apt installed pcl
find_package(PCL 1.8.1 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

message("PCL_LIBRARY_DIRS"  ${PCL_LIBRARY_DIRS})



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	message_generation
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	pluginlib
	pcl_conversions
	pcl_ros
	cv_bridge
	geometric_shapes
	moveit_visual_tools
  roscpp
  rospy
  roslint
  sensor_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

#message("{pcl_conversions}:" ${pcl_conversions_INCLUDE_DIRS})
#message("{pcl_ros}:" ${pcl_ros_INCLUDE_DIRS})



include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(point_cloud_grasp script/point_cloud_grasp.cpp)
add_dependencies(point_cloud_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_cloud_grasp ${catkin_LIBRARIES})

add_executable(pcl_publish script/pcl_publish.cpp)
add_dependencies(pcl_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_publish ${catkin_LIBRARIES})

add_executable(pcl_visualize2 script/pcl_visualize2.cpp)
add_dependencies(pcl_visualize2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_visualize2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(pointcloud_test script/pointcloud_test.cpp)
add_dependencies(pointcloud_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_test ${catkin_LIBRARIES})

add_executable(ur5_cloud_transformer script/ur5_cloud_transformer.cpp)
add_dependencies(ur5_cloud_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur5_cloud_transformer ${catkin_LIBRARIES})

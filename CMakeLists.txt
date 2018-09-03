cmake_minimum_required(VERSION 3.9)
project(universal_robot)

set(CMAKE_CXX_STANDARD 11)

add_executable(universal_robot
        ur5_moveit_config/script/pcl_publish.cpp
        ur5_moveit_config/script/pcl_publish_origin0.cpp
        ur5_moveit_config/script/pcl_visualize2.cpp
        ur5_moveit_config/script/point_cloud_grasp.cpp
        ur5_moveit_config/script/pointcloud_test.cpp
        ur5_moveit_config/script/ur5_cloud_transformer.cpp
        ur_kinematics/include/ur_kinematics/ikfast.h
        ur_kinematics/include/ur_kinematics/ur_kin.h
        ur_kinematics/include/ur_kinematics/ur_moveit_plugin.h
        ur_kinematics/src/ur_kin.cpp
        ur_kinematics/src/ur_kin_py.cpp
        ur_kinematics/src/ur_moveit_plugin.cpp)

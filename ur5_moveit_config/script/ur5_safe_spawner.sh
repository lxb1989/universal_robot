#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
rosrun ur5_moveit_config point_cloud_grasp  # motion planning with collision free
rosrun ur5_moveit_config pcl_publish   #publish point clouds from yaml
rosrun ur5_moveit_config ur5_cloud_transformer 
rosrun ur5_moveit_config pcl_visualize2   #subscribe point clouds and visulizer published

roslaunch ur5ur5_moveit_config pcOctomap.launch #convert point clouds to octomap

x-terminal-emulator -e roslaunch ur_gazebo ur5.launch & sleep 5 &&
x-terminal-emulator -e roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch  sim:=true & sleep 5 &&
x-terminal-emulator -e roslaunch ur5_moveit_config moveit_rviz.launch

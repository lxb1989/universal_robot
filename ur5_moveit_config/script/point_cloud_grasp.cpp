/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *

 *********************************************************************/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>
#include <tf/tf.h>
#include <math.h>

#define PI 3.14159265


/*
 * It will iterate through all the lines in file and
 * put them in given vector
 */
bool getFileContent(std::string fileName, std::vector<std::vector<double>> & vecGrasp)
{

	// Open the File
	std::ifstream in(fileName.c_str());

	// Check if object is valid
	if(!in)
	{
		std::cerr << "Cannot open the File : "<<fileName<<std::endl;
		return false;
	}

	std::string tmpstr;

	// Read the next line from File untill it reaches the end.
  int count = 0;

  std::regex pat_regex("(^-?[1-9]\\d*\\.\\d+|-?0\\.\\d+[1-9]\\d+)");  //regex expression, float type  ^(-?\\d+)(\\.\\d+)?$   (^-?[1-9]\\d*.\\d*|-?0.\\d*[1-9]\\d*)
	while (std::getline(in, tmpstr))
	{
    std::vector<double> vecOfDoubles;
		// Line contains string of length > 0 then save it in vector
    // std::cout << "tmpstr size:" << tmpstr.size() << std::endl;
    std::cout << "tmpstr:" << tmpstr<< std::endl;
		if(tmpstr.size() > 0)
		{
      std::regex_iterator<std::string::iterator> it(tmpstr.begin(), tmpstr.end(), pat_regex), end_it;
      std::cout << "it:" << it->str() << std::endl;
      // std::cout << "endit:" << *end_it << std::endl;
      for (; it != end_it; ++it) {  //regex match
           // std::cout << "test:" << it->str() << " ";  //输出匹配成功的数据
           vecOfDoubles.push_back(stod(it->str()));  //convert string to double , and push back double vector
       }
    }

		std::cout << "vecofDouble from file (xyz normal):" << vecOfDoubles[0] << ' '<< vecOfDoubles[1] << ' '<< vecOfDoubles[2] << ' '
		<< vecOfDoubles[3] << ' '<< vecOfDoubles[4] << ' '<< vecOfDoubles[5] << ' '<< std::endl;
    vecGrasp.push_back(vecOfDoubles);
    count++;
    // std::cout << "vecOfDoubles size:" << vecOfDoubles.size() << std::endl;
	}

  std::cout << "vecGrasp size:" << vecGrasp.size() << std::endl;
	//Close The File
	in.close();
	return true;
}

tf::Quaternion RPYToQuaternion(float R, float P, float Y)
{
  tf::Matrix3x3 mat;
  mat.setEulerYPR(Y,P,R);

  tf::Quaternion quat;
  mat.getRotation(quat);

  return quat;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_grasp");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //initialize
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world"); //, "/rviz_visual_markers"
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "grasp Demo", rvt::WHITE, rvt::XLARGE);
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // grasp pose list
  // get grasp pose from grasp.text
  std::vector<std::vector<double>>  vecGrasp;
  bool result = getFileContent("/home/yuezhen/catkin_ws/src/universal_robot/ur5_moveit_config/script/grasp.txt", vecGrasp);
  std::vector<geometry_msgs::Pose> grasp_pose_list;
  for (int i = 0; i < vecGrasp.size(); ++i)
  {
    geometry_msgs::Pose target_pose;
    //convert normal direction to axis and angle
		double v_length_square = pow(vecGrasp[i][4],2) + pow(vecGrasp[i][3], 2);
		double v_x = -vecGrasp[i][4] / sqrt(v_length_square) ;//components x of axis , and normalize
		double v_y = vecGrasp[i][3] / sqrt(v_length_square); //components y of axis , components v_z = 0 , and normalize

		double angle = acos(vecGrasp[i][5]);
		std::cout << "normal_z:" << vecGrasp[i][5] << std::endl;
		std::cout << "angle:" << angle << std::endl;

		std::cout << "grasp pose xyz  v_x v_y angle:" << vecGrasp[i][0] << ' '<< vecGrasp[i][1] << ' '<< vecGrasp[i][2] << ' '
		<< v_x << ' '<< v_y  << ' ' << angle<<std::endl;
    // tf::Quaternion qt = RPYToQuaternion(r, p, y);
    target_pose.orientation.w = cos(angle / 2);
    target_pose.orientation.x = v_x * sin(angle / 2);
    target_pose.orientation.y = v_y * sin(angle / 2);
    target_pose.orientation.z = 0;
    target_pose.position.x = vecGrasp[i][0];
    target_pose.position.y = vecGrasp[i][1];
    target_pose.position.z = vecGrasp[i][2];
    grasp_pose_list.push_back(target_pose);
  }


  // Planning to a grasp_pose_list , set home points for start pose , set goal points above grasp poses
  // home--goal--grasp
  // set home
  std::vector<double>  home_joints {-1.5707, -1.5707, 1.5707, -1.5707,-1.5707, 1.5707};

  for (int j = 0; j < grasp_pose_list.size(); ++j)
  {
    // plan from current pose to home;
    move_group.setStartStateToCurrentState();
    moveit::core::RobotStatePtr current_state =move_group.getCurrentState();
    std::vector<double> current_joint_positions;
    current_state->copyJointGroupPositions(joint_model_group,current_joint_positions);
    if (fabs(home_joints[0] - current_joint_positions[0]) > 0.001 || fabs(home_joints[1] - current_joint_positions[1]) > 0.001)
    {
      move_group.setJointValueTarget(home_joints);
      moveit::planning_interface::MoveGroupInterface::Plan home_plan;
      bool success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			ROS_INFO("Visualizing plan to target: %s",
							 success ? "SUCCEEDED" : "FAILED");

			// visualize the plan in Rviz.
			ROS_INFO("Visualizing plan to home ");
			visual_tools.deleteAllMarkers();
			visual_tools.publishText(text_pose, "Home Pose", rvt::WHITE, rvt::XLARGE);
			visual_tools.publishTrajectoryLine(home_plan.trajectory_, joint_model_group);
			visual_tools.trigger();
			// visual_tools.prompt("next step");
      move_group.execute(home_plan);
    }


    //plan for home--goal
    // set goal
    geometry_msgs::Pose target_pose = grasp_pose_list[j];
		geometry_msgs::Pose goal_pose = target_pose;
		double goal_delta = 0.1;
		goal_pose.position.x = target_pose.position.x - goal_delta * vecGrasp[j][3];
		goal_pose.position.y = target_pose.position.y - goal_delta * vecGrasp[j][4];
    goal_pose.position.z = target_pose.position.z - goal_delta * vecGrasp[j][5];
    move_group.setPoseTarget(goal_pose);
    moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
    bool success = (move_group.plan(goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		ROS_INFO("Visualizing plan to target: %s",
						 success ? "SUCCEEDED" : "FAILED");

		// visualize the plan in Rviz.
		ROS_INFO("Visualizing home to goal ");
		visual_tools.deleteAllMarkers();
		visual_tools.publishAxisLabeled(goal_pose, "goal_pose");
		visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(goal_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
		// visual_tools.prompt("next step");
    move_group.execute(goal_plan);
	  visual_tools.prompt("next step");



    // plan for goal--grasp
		geometry_msgs::Pose grasp_pose = target_pose;
		double grasp_delta = 0.008;
		grasp_pose.position.x = target_pose.position.x - grasp_delta * vecGrasp[j][3];
		grasp_pose.position.y = target_pose.position.y - grasp_delta * vecGrasp[j][4];
    grasp_pose.position.z = target_pose.position.z - grasp_delta * vecGrasp[j][5];
    move_group.setPoseTarget(grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    bool grasp_success = (move_group.plan(grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		ROS_INFO("Visualizing plan to target: %s",
						 grasp_success ? "SUCCEEDED" : "FAILED");
    if(grasp_success)
		{
			std::cout << "grasp pose " << j << "plan successful without collision";
		}
		// visualize the plan in Rviz.
		ROS_INFO("Visualizing goal to grasp ");
		visual_tools.deleteAllMarkers();
		visual_tools.publishAxisLabeled(grasp_pose, "grasp_pose");
		visual_tools.publishAxisLabeled(target_pose, "target_pose");
		visual_tools.publishText(text_pose, "Grasp Pose", rvt::WHITE, rvt::XLARGE);
		visual_tools.publishTrajectoryLine(grasp_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
		visual_tools.prompt("next step");
    move_group.execute(grasp_plan);

		ROS_INFO("Visualizing goto home ");
		visual_tools.prompt("next step: grasp pose finished");
  }
  // geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  // ROS_INFO_NAMED("tutorial", "current pose z: %f", current_pose.pose.position.z);
  // move_group.setStartStateToCurrentState();
  // geometry_msgs::Pose target_pose = current_pose.pose;
  // target_pose.position.z = current_pose.pose.position.z + 0.08;
  // ROS_INFO_NAMED("tutorial", "target pose z: %f", target_pose.position.z);
  // move_group.setPoseTarget(target_pose);
  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group
  // // to actually move the robot.
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //
  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //
  // ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal): %s", success ? "SUCCEEDED" : "FAILED");
  //
  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // We can also visualize the plan as a line with markers in RViz.
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(target_pose, "pose");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // move_group.move();


  ros::shutdown();
  return 0;
}

#include <ros/ros.h>
// PCL specific includes
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <pcl_ros/transforms.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_publish", 50);
  ros::Publisher grasp_pub = nh.advertise<sensor_msgs::PointCloud2> ("grasp_publish", 50);

    //for point cloud  in kinect_link
  cv::FileStorage fr("/home/wanfang/catkin_ws/src/universal_robot/ur5_moveit_config/config/scene.yaml", cv::FileStorage::READ);
  cv::Mat scene;
  fr["scene"]>>scene;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZ>);
  for(int ki=0;ki < scene.rows;ki++)
  {
      pcl::PointXYZ pointXYZ;

      pointXYZ.x = scene.at<float>(ki,0)/1000;
      pointXYZ.y = scene.at<float>(ki,1)/1000;
      pointXYZ.z = scene.at<float>(ki,2)/1000;
      cloud_scene->points.push_back(pointXYZ);
  }
  fr.release();

  // add nosie
  bool addNoise = false;
  if(addNoise)
  {
      float standard_deviation = 0.025;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
      // sensor_msgs::PointCloud2::Ptr noisy_buffer_ (new sensor_msgs::PointCloud2);

      // Create the filtering object
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_scene);
      sor.setLeafSize (0.02, 0.02, 0.02);
      sor.filter (*cloud_out);

      cloud_filtered->points.resize (cloud_out->points.size ());
      cloud_filtered->header = cloud_out->header;
      cloud_filtered->width = cloud_out->width;
      cloud_filtered->height = cloud_out->height;


      boost::mt19937 rng;
      rng.seed (static_cast<unsigned int> (time (0)));
      boost::normal_distribution<> nd (0, standard_deviation);
      boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

      for (size_t i = 0; i < cloud_out->points.size (); ++i)
      {
        cloud_filtered->points[i].x = cloud_out->points[i].x + static_cast<float> (var_nor ());
        cloud_filtered->points[i].y = cloud_out->points[i].y + static_cast<float> (var_nor ());
        cloud_filtered->points[i].z = cloud_out->points[i].z + static_cast<float> (var_nor ());
        // cloud_filtered->points[i].r = cloud_out->points[i].r;
        // cloud_filtered->points[i].g = cloud_out->points[i].g;
        // cloud_filtered->points[i].b = cloud_out->points[i].b;
      }


      *cloud_out = *cloud_filtered + *cloud_scene;
      *cloud_scene = *cloud_out;

      // pcl::toROSMsg (*cloud_out, *noisy_buffer_);
  }

  sensor_msgs::PointCloud2 cloudPoints;
  pcl::toROSMsg(*cloud_scene, cloudPoints);

  cloudPoints.header.stamp = ros::Time::now();
  cloudPoints.header.frame_id = "kinect_depth_optical_frame";
  // cloudPoints.header.frame_id = "kinect_link";
  // cloudPoints.height = cloudPoints.width = 1;

  //for grasp point
  std::string fileName2 = "/home/wanfang/catkin_ws/src/universal_robot/ur5_moveit_config/config/grasp.yaml";
  cv::FileStorage fs2(fileName2, cv::FileStorage::READ);
  cv::Mat pcGrasp;
  cv::Mat pcGraspNormal;
  fs2["grasp"]>>pcGrasp;
  fs2["normal"]>>pcGraspNormal;
  pcl::PointCloud<pcl::PointNormal>::Ptr grasp_cloud(new pcl::PointCloud<pcl::PointNormal>);
  std::vector<std::vector<double>>  grasp_poses;

  for(int ki=0;ki < pcGrasp.rows;ki++)
  {
      pcl::PointNormal pointNormal;
      std::vector<double> grasp_pose(6);
      pointNormal.x = pcGrasp.at<float>(ki,0)/1000;
      pointNormal.y = pcGrasp.at<float>(ki,1)/1000;
      pointNormal.z = pcGrasp.at<float>(ki,2)/1000;

      grasp_pose[0] = pointNormal.x;
      grasp_pose[1] = pointNormal.y;
      grasp_pose[2] = pointNormal.z;


      pointNormal.normal_x = pcGraspNormal.at<float>(ki,0);
      pointNormal.normal_y = pcGraspNormal.at<float>(ki,1);
      pointNormal.normal_z = pcGraspNormal.at<float>(ki,2);

      grasp_pose[3] = pointNormal.normal_x;
      grasp_pose[4] = pointNormal.normal_y;
      grasp_pose[5] = pointNormal.normal_z;
      grasp_cloud->points.push_back(pointNormal);
      grasp_poses.push_back(grasp_pose);

  }


  sensor_msgs::PointCloud2 grasp;
  pcl::toROSMsg(*grasp_cloud, grasp);

  grasp.header.stamp = ros::Time::now();
  grasp.header.frame_id = "kinect_depth_optical_frame";
  // grasp.header.frame_id = "kinect_link";



  //transform point cloud and grasp point from kinect_link to world
  tf::TransformListener listener_;
  sensor_msgs::PointCloud2 cloud_buffer_, grasp_buffer_;
  listener_.waitForTransform("world", "kinect_depth_optical_frame", ros::Time::now(), ros::Duration(1.0));
  pcl_ros::transformPointCloud("world", cloudPoints, cloud_buffer_, listener_);
  pcl_ros::transformPointCloud("world", grasp, grasp_buffer_, listener_);
  cloud_buffer_.header.stamp = ros::Time::now();
  cloud_buffer_.header.frame_id = "world";
  grasp_buffer_.header.stamp = ros::Time::now();
  grasp_buffer_.header.frame_id = "world";

  //output  the point clouds in world frame to cloud.text
  pcl::PointCloud<pcl::PointXYZ> cloud_base;
  pcl::fromROSMsg(cloud_buffer_, cloud_base);
  std::ofstream cloudfile("/home/wanfang/catkin_ws/src/universal_robot/ur5_moveit_config/script/cloud.txt", std::ios_base::out);
  for (int ic = 0; ic < cloud_base.size(); ++ic)
  {
    cloudfile << cloud_base.points[ic].x << ' ' <<cloud_base.points[ic].y << ' ' << cloud_base.points[ic].z << ' ' << std::endl;
  }

  cloudfile.close();

  // output the grasp pose to grasp.txt
  pcl::PointCloud<pcl::PointNormal> grasp_base;
  pcl::fromROSMsg(grasp_buffer_, grasp_base);
  std::ofstream  graspfile("/home/wanfang/catkin_ws/src/universal_robot/ur5_moveit_config/script/grasp.txt", std::ios_base::out);
  for (int ig = 0; ig < grasp_base.size(); ++ig)
  {
    graspfile << grasp_base.points[ig].x << ' ' <<grasp_base.points[ig].y << ' ' << grasp_base.points[ig].z << ' ' <<
    grasp_base.points[ig].normal_x << ' ' << grasp_base.points[ig].normal_y << ' ' <<grasp_base.points[ig].normal_z  << std::endl;
  }

  graspfile.close();



  ros::Rate loop_rate(1.0);
  while(nh.ok())
  {
     // cloudPoints.header.stamp = ros::Time::now();
     // cloudPoints.header.stamp = ros::Time::now();
     // cloudPoints.header.frame_id = "world";
     // pub.publish(cloudPoints);
     // grasp_pub.publish(grasp);
     pub.publish(cloud_buffer_);
     grasp_pub.publish(grasp_buffer_);
     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;

}

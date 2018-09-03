#include <ros/ros.h>
// PCL specific includes
#include <opencv2/core/core.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/MarkerArray.h>

// void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_scene)
// {
//   ros::Rate loop_rate(4);
//   while(nh.ok())
//   {
//      pcl_conversions::toPCL(ros::Time::now(), cloud_scene->header.stamp);
//      pub.publish(cloud_scene);
//      ros::spinOnce();
//      loop_rate.sleep();
//   }
// }

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_publish", 10);
  ros::Publisher grasp = nh.advertise<<visualization_msgs::Marker> ("grasp_publish", 10);

    //for point cloud
  cv::FileStorage fr("/home/yuezhen/catkin_ws/src/universal_robot/ur5_moveit_config/script/test/scene.yaml", cv::FileStorage::READ);
  cv::Mat scene;
  fr["scene"]>>scene;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZ>);
  for(int ki=0;ki < scene.rows;ki++)
  {
      pcl::PointXYZ pointXYZ;

      pointXYZ.x = scene.at<float>(ki,0);
      pointXYZ.y = scene.at<float>(ki,1);
      pointXYZ.z = scene.at<float>(ki,2);
      cloud_scene->points.push_back(pointXYZ);
  }
  fr.release();

  sensor_msgs::PointCloud2 cloudPoints;
  pcl::toROSMsg(*cloud_scene, cloudPoints);

  cloudPoints.header.stamp = ros::Time::now();
  cloudPoints.header.frame_id = "world";
  // cloudPoints.height = cloudPoints.width = 1;

  //for grasp point
  std::string fileName2 = "/home/yuezhen/catkin_ws/src/universal_robot/ur5_moveit_config/script/test/scene.yaml";
  cv::FileStorage fs2(fileName2, cv::FileStorage::READ);
  cv::Mat pcGrasp;
  cv::Mat pcGraspNormal;
  fs2["grasp"]>>pcGrasp;
  fs2["normal"]>>pcGraspNormal;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_grasp(new pcl::PointCloud<pcl::PointXYZ>);
  visualization_msgs::MarkerArray arrow;
  for(int ki=0;ki < pcGrasp.rows;ki++)
  {
      pcl::PointXYZ pointXYZ;
      pcl::Normal pointNormal;
      // geometry_msgs::point pos;

      pointXYZ.x = pcGrasp.at<float>(ki,0);
      pointXYZ.y = pcGrasp.at<float>(ki,1);
      pointXYZ.z = pcGrasp.at<float>(ki,2);

      // pos.x = pcGrasp.at<float>(ki,0);
      // pos.y = pcGrasp.at<float>(ki,1);
      // pos.z = pcGrasp.at<float>(ki,2);

      pointNormal.normal_x = pcGraspNormal.at<float>(ki,0);
      pointNormal.normal_y = pcGraspNormal.at<float>(ki,1);
      pointNormal.normal_z = pcGraspNormal.at<float>(ki,2);

      pcl::PointCloud<pcl::PointXYZ>::Ptr s1(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::Normal>::Ptr s2(new pcl::PointCloud<pcl::Normal>);

      s1->push_back(pointXYZ);
      s2->push_back(pointNormal);
       viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(s1, s2, 20,50, std::to_string(ki)+"normals",0);
      pcl::PointXYZ m2;
      m2.x = (pointXYZ.x+40*pointNormal.normal_x);
      m2.y = (pointXYZ.y+40*pointNormal.normal_y);
      m2.z = (pointXYZ.z+40*pointNormal.normal_z);

      viewer.addArrow<pcl::PointXYZ> (m2,pointXYZ,0,1,0,false,std::to_string(ki)+"arrow");
     // arrow.markers[ki].id = ki;
     // arrow.markers[ki].type = visualization_msgs::Marker::ARROW;
     // arrow.markers[ki].action = visualization_msgs::Marker::ADD;
     //
     // arrow.markers[ki].pose.position.x = pointXYZ.x;
     // arrow.markers[ki].pose.position.y = pointXYZ.y;
     // arrow.markers[ki].pose.position.z = pointXYZ.z;
     //
     // arrow.markers[ki].pose.orientation.x = 45;
     // arrow.markers[ki].pose.orientation.y = 0.0;
     // arrow.markers[ki].pose.orientation.z = 45;
     // arrow.markers[ki].pose.orientation.w = 1.0;
     //
     // arrow.markers[ki].scale.x= 5;
     // arrow.markers[ki].scale.y= 0.1;
     // arrow.markers[ki].scale.z = 0.1;
     //
     // arrow.markers[ki].color.g = 0.0f;
     // arrow.markers[ki].color.a = 1.0;
     // arrow.markers[ki].color.r = 0.0f;
     // arrow.markers[ki].color.b = 1.0f;
     //
     // arrow.markers[ki].lifetime = ros::Duration();
     // arrow.markers[ki].header.frame_id = "world";
     // arrow.markers[ki].header.stamp = ros::Time::now();
  }

  ros::Rate loop_rate(4);
  while(nh.ok())
  {
     cloudPoints.header.stamp = ros::Time::now();
     pub.publish(cloudPoints);
     pub.publish(arrow);
     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;

}

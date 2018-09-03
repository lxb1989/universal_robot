#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");
  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud_test", 50);
  unsigned int num_points = 100;
  ros::Rate r(1.0);
  while(n.ok()){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize(num_points);
    //generate some fake data for our point cloud
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 0.1*i;
      cloud.points[i].y = 0.1*i;
      cloud.points[i].z = 5;
      // cloud.channels[0].values[i] = 255;
    }

    sensor_msgs::PointCloud2 cloud_sensor;
    pcl::toROSMsg(cloud, cloud_sensor);
    cloud_sensor.header.stamp = ros::Time::now();
    cloud_sensor.header.frame_id = "world";
    //we'll also add an intensity channel to the cloud
    // cloud.channels.resize(1);
    // cloud.channels[0].name = "rgb";
    // cloud.channels[0].values.resize(num_points);



    cloud_pub.publish(cloud_sensor);
    r.sleep();
  }
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

class cloudHandler
{
public:
    cloudHandler()
    : viewer("Cloud Viewer")
    {
        output_sub = nh.subscribe("pcl_publish", 1, &cloudHandler::outputCB, this);
        // output_sub = nh.subscribe("/ur5/world/points", 1, &cloudHandler::outputCB, this);
        outputGrasp_sub = nh.subscribe("grasp_publish", 1, &cloudHandler::outputGrasp, this);
        // downsampled_sub = nh.subscribe("pcl_partitioned", 1, &cloudHandler::downsampledCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);

        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, output_view);
        viewer.setBackgroundColor(0, 0, 0, output_view);

        // viewer.createViewPort(0.5, 0.0, 1.0, 1.0, downsampled_view);
        // viewer.setBackgroundColor(0, 0, 0, downsampled_view);

        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();
    }

    void outputCB(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*input, cloud);

        viewer.removeAllPointClouds(output_view);
        viewer.addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "output", output_view);
    }

    void outputGrasp(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        pcl::PointCloud<pcl::PointNormal> cloudNormal;
        pcl::fromROSMsg(*input, cloudNormal);

        for (int i = 0; i < cloudNormal.size(); ++i)
        {
          pcl::PointXYZ m1, m2;
          pcl::Normal pointNormal;

          m1.x = cloudNormal[i].x;
          m1.y = cloudNormal[i].y;
          m1.z = cloudNormal[i].z;

          pointNormal.normal_x = cloudNormal[i].normal_x;
          pointNormal.normal_y = cloudNormal[i].normal_y;
          pointNormal.normal_z = cloudNormal[i].normal_z;


          viewer.addSphere(m1,0.005,1,0,0,std::to_string(i)+"grasp",output_view);

          m2.x = (m1.x - 0.04*pointNormal.normal_x);
          m2.y = (m1.y - 0.04*pointNormal.normal_y);
          m2.z = (m1.z - 0.04*pointNormal.normal_z);

          viewer.addArrow<pcl::PointXYZ>(m2,m1,0,1,0,false,std::to_string(i)+"arrow",output_view);
        }
    }
    //
    // void downsampledCB(const sensor_msgs::PointCloud2ConstPtr& input)
    // {
    //     pcl::PointCloud<pcl::PointXYZ> cloud;
    //     pcl::fromROSMsg(*input, cloud);
    //
    //     viewer.removeAllPointClouds(downsampled_view);
    //     viewer.addPointCloud<pcl::PointXYZ>(cloud., "downsampled", downsampled_view);
    // }

    void timerCB(const ros::TimerEvent&)
    {
        viewer.spinOnce();

        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber output_sub;// downsampled_sub;
    ros::Subscriber outputGrasp_sub;
    pcl::visualization::PCLVisualizer viewer;
    int output_view;// downsampled_view;
    ros::Timer viewer_timer;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_visualize2");

    cloudHandler handler;

    ros::spin();

    return 0;
}

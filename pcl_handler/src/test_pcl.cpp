#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "pcl/filters/impl/filter.hpp"

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*input, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB>* cloud_dense;
  pcl::PointCloud<pcl::PointXYZRGB>* cloud_dense_out;
  pcl::fromPCLPointCloud2(*cloud, *cloud_dense);

  std::vector<int>* indice;
  pcl::removeNaNFromPointCloud(*cloud_dense, *cloud_dense_out, *indice);

  pcl::toPCLPointCloud2(*cloud_dense_out, *cloud); 	


  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud, output);

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}

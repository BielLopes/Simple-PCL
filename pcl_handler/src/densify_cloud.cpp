// #include <ros/ros.h>
// // PCL specific includes
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
#include <pcl/filters/impl/filter.hpp>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>


struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  // pcl_conversions::toPCL(*input, *cloud);


  // pcl::PointCloud<pcl::PointXYZIRT>* cloud_dense = new pcl::PointCloud<pcl::PointXYZIRT>;
  // pcl::fromPCLPointCloud2(*cloud, *cloud_dense);

  pcl::PointCloud<PointXYZIRT>::Ptr cloud_dense;
  // pcl::PointCloud<pcl::PointXYZIRT>* cloud_dense = new pcl::PointCloud<pcl::PointXYZIRT>;
  pcl::fromROSMsg(*input, *cloud_dense);

  std::vector<int>* indice = new std::vector<int>;
  pcl::removeNaNFromPointCloud(*cloud_dense, *cloud_dense, *indice);

  // pcl::toPCLPointCloud2(*cloud_dense, *cloud); 	


  sensor_msgs::PointCloud2 output = sensor_msgs::PointCloud2();
  pcl::toROSMsg(*cloud_dense, output);
  // pcl_conversions::fromPCL(*cloud, output);


  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_converter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_dense", 1);
  // Spin
  ros::spin ();
}

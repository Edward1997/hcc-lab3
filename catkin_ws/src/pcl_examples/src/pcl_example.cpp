#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "pcl_ros/point_cloud.h"

ros::Publisher centroid_point_publisher;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

	PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
	pcl::fromROSMsg (*input, *cloud); //convert from PointCloud2 to pcl point type
  //Exmaple : pcl PointCloudXYZRGB information
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  printf("-------------------------Cloud information-----------------------------\n");
  printf("cloud size: %d\n",cloud->points.size());
  int cloud_size=cloud->points.size();
  printf("The first cloud coordinate and color information:\n");
  printf("X: %4lf, Y: %4lf, Z: %4lf\n",cloud->points[0].x,cloud->points[0].y,cloud->points[0].z);
  printf("The last cloud coordinate and color information:\n");
  printf("X: %4lf, Y: %4lf, Z: %4lf,\n",cloud->points[cloud_size-1].x,cloud->points[cloud_size-1].y,cloud->points[cloud_size-1].z);
  printf("**********************************************************************\n");
  
  //Exercise : Calculate the centroid of the object (coordinate average)
  double x=0; double y=0;double z=0;
  ///Write your algorithm/////////////////

	
	
	
	
	
	
	
	

  /////////////////////////////////////////
  geometry_msgs::PointStamped point;
  point.header.frame_id = "/camera_depth_optical_frame";
  point.header.stamp = ros::Time();
  point.point.x = x;
  point.point.y = y;
  point.point.z = z;
  centroid_point_publisher.publish(point); 

  printf("The model location:\n");
  printf("X: %4lf, Y: %4lf, Z: %4lf\n",x,y,z);

}   
int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber model_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
     centroid_point_publisher = nh.advertise<geometry_msgs::PointStamped> ("/camera/centroid", 1);
     // Spin
     ros::spin ();
  }

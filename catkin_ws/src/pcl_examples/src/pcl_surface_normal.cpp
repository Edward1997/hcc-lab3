#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/common/common.h>


ros::Publisher surface_normal_vis_publisher; 
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;


void  surface_normal_vis (PointCloudXYZ::Ptr point_cloud, PointCloudNormal::Ptr normal);
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  printf("Start calculating surface normal. \n");
  double scale = 0.02;
	PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
  PointCloudXYZ::Ptr cloud_no_nan(new PointCloudXYZ);
	pcl::fromROSMsg (*input, *cloud); //convert from PointCloud2 to pcl point type
  //Exmaple : pcl PointCloudXYZRGB information
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nan, indices);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_no_nan);
  sor.setLeafSize (0.005f, 0.0005f, 0.005f);
  sor.filter (*cloud_no_nan);

 // pcl::NormalEstimation<PointCloudXYZ, PointCloudNormal> ne;
   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_no_nan);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> );
  ne.setSearchMethod (tree);
  PointCloudNormal::Ptr cloud_normals (new PointCloudNormal);
  ne.setRadiusSearch (scale);
  ne.compute (*cloud_normals);
  printf("Finish calculating surface normal. \n");
  surface_normal_vis (cloud_no_nan,cloud_normals);
  /*
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
  for(int i=0;i<cloud->points.size();i++){

    x+=cloud->points[i].x;
    y+=cloud->points[i].y;
    z+=cloud->points[i].z;
  }
  x /=cloud->points.size();
  y /=cloud->points.size();
  z /=cloud->points.size();

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
  */
}   

void  surface_normal_vis (PointCloudXYZ::Ptr point_cloud, PointCloudNormal::Ptr normal){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/camera_depth_optical_frame";
    line_list.header.stamp = ros::Time::now();
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.001;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    // Create the vertices for the points and lines
    for (int i = 0; i < point_cloud->points.size(); i=i+10)
    {
      geometry_msgs::Point p1; 
      p1.x = point_cloud->points[i].x;
      p1.y = point_cloud->points[i].y;
      p1.z = point_cloud->points[i].z;
      geometry_msgs::Point p2;
      p2.x = p1.x + normal->points[i].normal[0]*0.01;
      p2.y = p1.y + normal->points[i].normal[1]*0.01;
      p2.z = p1.z + normal->points[i].normal[2]*0.01;

 
      // The line list needs two points for each line
      line_list.points.push_back(p1);
      line_list.points.push_back(p2);
    }
    surface_normal_vis_publisher.publish(line_list);
    printf("Publish Successfully\n");
}

int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber model_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
     surface_normal_vis_publisher = nh.advertise<visualization_msgs::Marker>("/camera/surface_normal", 10);
     // Spin
     ros::spin ();
  }

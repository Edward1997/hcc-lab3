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
  //Remove NAN from the point cloud
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nan, indices);

  //Downsample the point cloud 
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_no_nan);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_no_nan);

  //////////////////////Task 1 : Complete cloud_cb()//////////////////////    
  //Please use PCL to calculate surface normal
  //Input Point Cloud: cloud_no_nan, type: pcl::PointCloud<pcl::PointXYZ> 
  //Output Surface normal: cloud_normals, type: pcl::PointCloud<pcl::Normal>





   
  ///////////////////////////////////////////////////////////////////////
  printf("Finish calculating surface normal. \n");
  surface_normal_vis (cloud_no_nan,cloud_normals);
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

    //////////////////////Task 2 : Complete surface_normal_vis()///////////    
    //////Please assign surface normal to RVIZ marker "line_list"//////////
   
    for (int i = 0; i < point_cloud->points.size(); i=i+10)
    {
      //Create point1: The point cloud




      //Create point2: The surface normal




 
      // The line list needs two points for each line (Fill the blank)
      line_list.points.push_back("XXX");
      line_list.points.push_back("XXX");
    }
    ////////////////////////////////////////////////////////////////////////
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

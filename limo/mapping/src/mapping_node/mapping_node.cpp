#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<iostream>
#include "mapping.h"

 
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "mapping_node");
  ros::NodeHandle nh;
  ros::NodeHandle privateNode("~");
 
  LAM_Mapping lidarmap;
  if(lidarmap.setup(nh,privateNode))
  {
     lidarmap.spin();
  }
     

  return 0;
}
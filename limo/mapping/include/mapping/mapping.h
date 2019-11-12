#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include<iostream>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include<fstream>

class LAM_Mapping
{
    
    public:
       LAM_Mapping();
       ~LAM_Mapping();

       void PathHandler(const nav_msgs::Path::ConstPtr& lidarPath);
       void CloudPointHandler(const sensor_msgs::PointCloud2ConstPtr& srccloud );
       bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);
       void mapping();
       void savepath();
       void spin();

    private:
       
       ros::Publisher map_pub;

       ros::Subscriber cloud_sub;
       ros::Subscriber path_sub;

       pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
       pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud;

       std::ofstream path_file;
       
       //当前点云的位姿
       Eigen::Isometry3d Twc;

       int framenum;

};


#endif
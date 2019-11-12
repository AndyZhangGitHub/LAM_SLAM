#include "mapping.h"
#include <ros/ros.h>
#include<iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include<fstream>
using namespace std;


//构造函数
LAM_Mapping::LAM_Mapping():framenum(0),cur_cloud(new pcl::PointCloud<pcl::PointXYZI>()),map_cloud(new pcl::PointCloud<pcl::PointXYZI>()),Twc(Eigen::Isometry3d::Identity())
{
     //map_cloud = NULL;
     //cur_cloud = NULL;
     //Twc = Eigen::Isometry3d::Identity();
     
     path_file.open("/home/Andy/catkin_limo_ws/src/limo/mapping/path.txt");

}
LAM_Mapping::~LAM_Mapping()
{
    path_file.close();
}
void LAM_Mapping::spin()
{
    
  ros::spin();
}

bool LAM_Mapping::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
    std::cout<<"--------------设置相关参数---------------------------"<<std::endl;
    map_pub = node.advertise<sensor_msgs::PointCloud2>("mapping_cloud", 1);

    cloud_sub = node.subscribe("/sensor/velodyne/cloud_euclidean", 2, &LAM_Mapping::CloudPointHandler,this);
    path_sub  = node.subscribe("/estimate/complete_path", 1, &LAM_Mapping::PathHandler,this);

    return true;


}
void LAM_Mapping::CloudPointHandler(const sensor_msgs::PointCloud2ConstPtr& srccloud)
{

    // std::cout<<"1.接收点云数据---------------------------"<<std::endl;
    //framenum++;
     pcl::fromROSMsg(*srccloud,*cur_cloud);
    
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
        
    static pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize( 0.1, 0.1, 0.1 );
    voxel.setInputCloud( cur_cloud );
    
    voxel.filter( *cur_cloud );

     pcl::transformPointCloud( *cur_cloud, *output, Twc.matrix() );
     *map_cloud += *output;
    
    std::cout<<"--------------framecloud size："<<output->size()<<std::endl;
    std::cout<<"--------------mapcloud size  ："<<map_cloud->size()<<std::endl<<std::endl;
     
     sensor_msgs::PointCloud2 cloudmap_msg;
     pcl::toROSMsg(*map_cloud,cloudmap_msg);

     cloudmap_msg.header.frame_id = "estimate/local_cs_vehicle";
     map_pub.publish(cloudmap_msg);
     
      
}

void LAM_Mapping::PathHandler(const nav_msgs::Path::ConstPtr& lidarPath)
{
      double t_x,t_y,t_z;
     
      //std::cout<<"2.接收轨迹数据---------------------------"<<std::endl;
      geometry_msgs::PoseStamped new_pose = lidarPath->poses.back();
      
      t_x = new_pose.pose.position.x;
      t_y = new_pose.pose.position.y;
      t_z = new_pose.pose.position.z;

      path_file<< t_x<<" "<<t_y<<" "<<t_z<<std::endl;

      //用四元数表示的旋转
      Eigen::Quaterniond Q(new_pose.pose.orientation.w, new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z);
           
      Eigen::Isometry3d  T(Q);
      T.pretranslate(Eigen::Vector3d(t_x,t_y,t_z));
      Twc = T;

}



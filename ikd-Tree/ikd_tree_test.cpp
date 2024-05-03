/*
    Description: An example to introduce box search and radius search using ikd-Tree
    Author: Hyungtae Lim, Yixi Cai
    Modified by xxl1998 May,2024
*/
#include "ikd_Tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>
#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>


using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;
template class KD_TREE<pcl::PointXYZ>;

KD_TREE<PointType> ikdtree;
int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "ikd_tree_test");
    ros::NodeHandle nh;

    /*** Load point cloud data */
    pcl::PointCloud<PointType>::Ptr src(new pcl::PointCloud<PointType>);
    string filename = ROOT_DIR;
    filename += "materials/hku_demo_pointcloud.pcd";
    if (pcl::io::loadPCDFile<PointType>(filename, *src) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file.\n");
        std::cout << filename << std::endl;
        return (-1);
    }
    printf("Original: %d points are loaded\n", static_cast<int>(src->points.size()));

    /*** Build ikd-Tree */
    auto start = chrono::high_resolution_clock::now();
    ikdtree.Build((*src).points);
    auto end      = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
    printf("Building tree takes: %0.3f ms\n", float(duration) / 1e3);
    printf("# of valid points: %d \n", ikdtree.validnum());

    // Nearest Search
    PointVector search_result;
    vector<float> PointDist;
    PointType target;
    target.x = 0.0f;
    target.y = 0.0f;
    target.z = 5.0f;
    start = chrono::high_resolution_clock::now();
    ikdtree.Nearest_Search(target, 5, search_result, PointDist);
    end = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
    printf("Search nearest point time cost is %0.3f ms\n",float(duration)/1e3);

    ros::Publisher pub_pc_map = nh.advertise<sensor_msgs::PointCloud2>(
        "/pc_map", 1);
    sensor_msgs::PointCloud2 msg_pc_map;
    pcl::toROSMsg(*src, msg_pc_map);
    msg_pc_map.header.frame_id = "map";
    ros::Rate r(10.0f);

    while(ros::ok()){
        msg_pc_map.header.stamp = ros::Time::now();
        pub_pc_map.publish(msg_pc_map);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

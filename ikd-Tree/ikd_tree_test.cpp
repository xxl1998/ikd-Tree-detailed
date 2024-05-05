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
#include <jsk_recognition_msgs/PolygonArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Eigen>


using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;
template class KD_TREE<pcl::PointXYZ>;

#define NUM_MATCH_POINTS    (5)
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a,b)  Eigen::Matrix<double, (a), (b)>
#define VD(a)    Eigen::Matrix<double, (a), 1>
#define MF(a,b)  Eigen::Matrix<float, (a), (b)>
#define VF(a)    Eigen::Matrix<float, (a), 1>

template<typename T>
bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    Eigen::IOFormat matrix_print_format(4, 0, ", ", "\n", "[", "]");
    std::string sep = "\n----------------------------------------\n";
    std::cout << "A:" << std::endl << A.format(matrix_print_format) << sep;
    std::cout << "b:" << std::endl << b.format(matrix_print_format) << sep;
    std::cout << "x:" << std::endl << normvec.format(matrix_print_format) << sep;

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

KD_TREE<PointType> ikdtree;
int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "ikd_tree_test");
    ros::NodeHandle nh;

    // Create a TransformBroadcaster object
    tf2_ros::TransformBroadcaster broadcaster;

    // Create a TransformStamped message
    geometry_msgs::TransformStamped transformStamped;
    // Set the header
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";  // name of the parent frame
    transformStamped.child_frame_id = "world";    // name of the child frame
    // Set the transform
    transformStamped.transform.translation.x = 0.0;  // translation along x-axis
    transformStamped.transform.translation.y = 0.0;  // translation along y-axis
    transformStamped.transform.translation.z = 0.0;  // translation along z-axis
    transformStamped.transform.rotation.x = 0.0;     // quaternion x-component
    transformStamped.transform.rotation.y = 0.0;     // quaternion y-component
    transformStamped.transform.rotation.z = 0.0;     // quaternion z-component
    transformStamped.transform.rotation.w = 1.0;     // quaternion scalar-component

    /*** Load point cloud data */
    pcl::PointCloud<PointType>::Ptr pc_map(new pcl::PointCloud<PointType>);
    string root_dir = ROOT_DIR;
    string pcd_file_map = root_dir + "materials/map.pcd";
    if (pcl::io::loadPCDFile<PointType>(pcd_file_map, *pc_map) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file.\n");
        std::cout << pcd_file_map << std::endl;
        return (-1);
    }
    printf("Original: %d points are loaded\n", static_cast<int>(pc_map->points.size()));

    pcl::PointCloud<PointType>::Ptr pc_scan_body(new pcl::PointCloud<PointType>);
    string pcd_file_scan_body = root_dir + "materials/scan_body.pcd";
    if (pcl::io::loadPCDFile<PointType>(pcd_file_scan_body, *pc_scan_body) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file.\n");
        std::cout << pcd_file_scan_body << std::endl;
        return (-1);
    }
    printf("Original: %d points are loaded\n", static_cast<int>(pc_scan_body->points.size()));

    pcl::PointCloud<PointType>::Ptr pc_scan_world(new pcl::PointCloud<PointType>);
    string pcd_file_scan_world = root_dir + "materials/scan_world.pcd";
    if (pcl::io::loadPCDFile<PointType>(pcd_file_scan_world, *pc_scan_world) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file.\n");
        std::cout << pcd_file_scan_world << std::endl;
        return (-1);
    }
    printf("Original: %d points are loaded\n", static_cast<int>(pc_scan_world->points.size()));

    /*** Build ikd-Tree */
    auto start = chrono::high_resolution_clock::now();
    ikdtree.Build((*pc_map).points);
    auto end      = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
    printf("Building tree takes: %0.3f ms\n", float(duration) / 1e3);
    printf("# of valid points: %d \n", ikdtree.validnum());

    ros::Publisher pub_pc_map = nh.advertise<sensor_msgs::PointCloud2>(
        "/pc_map", 1);
    sensor_msgs::PointCloud2 msg_pc_map;
    pcl::toROSMsg(*pc_map, msg_pc_map);
    msg_pc_map.header.frame_id = "map";

    ros::Publisher pub_pc_scan_world = nh.advertise<sensor_msgs::PointCloud2>(
        "/pc_scan_world", 1);
    sensor_msgs::PointCloud2 msg_pc_scan_world;
    pcl::toROSMsg(*pc_scan_world, msg_pc_scan_world);
    msg_pc_scan_world.header.frame_id = "map";

    ros::Publisher pub_plane = nh.advertise<jsk_recognition_msgs::PolygonArray>(
        "/plane", 10);
    jsk_recognition_msgs::PolygonArray msg_polygon_array;
    geometry_msgs::PolygonStamped polygon_s;
    geometry_msgs::Point32 point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    polygon_s.polygon.points.push_back(point);
    point.x = 10.0;
    point.y = 0.0;
    point.z = 0.0;
    polygon_s.polygon.points.push_back(point);
    point.x = 10.0;
    point.y = 10.0;
    point.z = 0.0;
    polygon_s.polygon.points.push_back(point);
    polygon_s.header.frame_id = "map";
    polygon_s.header.stamp = ros::Time::now();
    msg_polygon_array.polygons.push_back(polygon_s);
    msg_polygon_array.header.frame_id = "map";

    ros::Rate r(10.0f);

    // scan point size, also loop number
    unsigned int scan_points_size = pc_scan_world->points.size();
    std::cout << "scan_points_size: " << scan_points_size << std::endl;

    // save all nearest search result
    std::vector<PointVector>  Nearest_Points;
    Nearest_Points.resize(scan_points_size);

    bool point_selected_surf[10000] = {0};

    while(ros::ok()){
        for (int i = 0; i < scan_points_size; i++)
        {
            PointType &point_body  = pc_scan_body->points[i];
            PointType &point_world = pc_scan_world->points[i];
            V3D p_body(point_body.x, point_body.y, point_body.z);
            PointVector &points_near = Nearest_Points[i];
            std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            /** Find the closest surfaces in the map **/
            start = chrono::high_resolution_clock::now();
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            end = chrono::high_resolution_clock::now();
            duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
            printf("Search nearest point time cost is %0.3f ms\n",float(duration)/1e3);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
            if (!point_selected_surf[i]) continue;

            VF(4) pabcd;
            point_selected_surf[i] = false;
            if (esti_plane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
                if (s > 0.9)
                {
                    point_selected_surf[i] = true;
                }
            }
            r.sleep();
        ros::spinOnce();
        }
        msg_pc_map.header.stamp = ros::Time::now();
        pub_pc_map.publish(msg_pc_map);
        msg_pc_scan_world.header.stamp = ros::Time::now();
        pub_pc_scan_world.publish(msg_pc_scan_world);
        msg_polygon_array.header.stamp = ros::Time::now();
        pub_plane.publish(msg_polygon_array);
        transformStamped.header.stamp = ros::Time::now();
        broadcaster.sendTransform(transformStamped);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}

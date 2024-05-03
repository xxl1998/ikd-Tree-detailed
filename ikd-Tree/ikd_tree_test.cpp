/*
    Description: An example to introduce box search and radius search using ikd-Tree
    Author: Hyungtae Lim, Yixi Cai
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


using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;
template class KD_TREE<pcl::PointXYZ>;

void colorize( const PointVector &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> &color) {
    int N = pc.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;

    for (int i = 0; i < N; ++i) {
        const auto &pt = pc[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

KD_TREE<PointType> ikdtree;
int main(int argc, char **argv) {
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

    /*** Below codes are just for visualization */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr searched_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::visualization::PointCloudColorHandlerGenericField<PointType> src_color(src, "x");
    colorize(search_result, *searched_colored, {255, 0, 0});

    pcl::visualization::PCLVisualizer viewer0("Nearest Search");
    viewer0.addPointCloud<PointType>(src,src_color, "src");
    viewer0.addPointCloud<pcl::PointXYZRGB>(searched_colored, "searched");
    viewer0.setCameraPosition(-5, 30, 175,  0, 0, 0, 0.2, -1.0, 0.2);
    viewer0.setSize(1600, 900);

    while (!viewer0.wasStopped() ){
        viewer0.spinOnce();
    }

    return 0;
}

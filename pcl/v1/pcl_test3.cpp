#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/opencv.hpp>
#include <pcl/compression/organized_pointcloud_conversion.h>



void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr &downsampled_output)
{
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox_grid.setInputCloud(cloud);
    vox_grid.filter(*downsampled_output);
}


void compute_surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float normal_radius, pcl::PointCloud<pcl::Normal>::Ptr &normals_output)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

    norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));

    norm_est.setRadiusSearch(normal_radius);

    norm_est.setInputCloud(cloud);

    norm_est.compute(*normals_output);

}


void visualize_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr normal_points, const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud(cloud, "points");
    viz.addPointCloud(normal_points, "normals_points");
    viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (normal_points, normals, 1, 0.01);
    viz.spin();
}











int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ> ("/home/julian/Capstone_2019/Capstone_2019-master/pcl-test/v1/depthpcl_2.ply", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }


    pcl::visualization::CloudViewer viewer ("Cluster viewer");
         viewer.showCloud(cloud);
  while (!viewer.wasStopped ())
  {
  }


   return (0);
}

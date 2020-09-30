#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

//using namespace std::chrono_literals;



int main (int argc, char** argv)
{
      pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/parallels/working_code/PCL_testing/PCL_LoadPlyFile/table_scene_lms400.pcd", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
    pcl::IndicesPtr indices (new std::vector <int>);
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1.0);
      pass.filter (*indices);

      pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
      reg.setInputCloud (cloud);
      reg.setIndices (indices);
      reg.setSearchMethod (tree);
      reg.setDistanceThreshold (10);
      reg.setPointColorThreshold (6);
      reg.setRegionColorThreshold (5);
      reg.setMinClusterSize (600);

      std::vector <pcl::PointIndices> clusters;
      reg.extract (clusters);

      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
      pcl::visualization::CloudViewer viewer ("Cluster viewer");
      viewer.showCloud (colored_cloud);
      while (!viewer.wasStopped ())
      {
        //std::this_thread::sleep_for(100us);
      }

      return (0);
}

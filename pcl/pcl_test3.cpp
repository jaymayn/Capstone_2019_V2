

//convert depth images into to pointclouds, write to pcd or ply - DONE YO!

//concentante multiple point clouds

//normal estimation and plan segementaion on final point cloud


#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/compression/organized_pointcloud_conversion.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>



int main (int argc, char** argv)
{

        cv::Mat depth_image;
        depth_image=cv::imread("/home/julian/box2_depth_image_raw_jpg/left0437.jpg",CV_LOAD_IMAGE_UNCHANGED);
        int rows=depth_image.rows;
        int cols=depth_image.cols;
        float focalLength = 525;
        // convert depth image to std::vector<uint16>
        std::vector<float> vec_image;
        // code to copy contents of matrix to vec

       // std::cout<<"rows: "<<rows<<" cols: "<<cols<<std::endl;

        for (int i=0; i< rows;i++)
        {
                for (int j=0; j< cols;j++)
                {
                        vec_image.push_back(depth_image.at<uchar>(i,j));
                }
        }

        std::vector<uchar> dummy;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
       // pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());



        // convert depth image to pointcloud

        pcl::io::OrganizedConversion<pcl::PointXYZ,false>::convert(vec_image, dummy, false,cols, rows, focalLength,*cloud);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate (Eigen::AngleAxisf (M_PI /*180°*/, Eigen::Vector3f::UnitX ()));

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
         // You can either apply transform_1 or transform_2; they are the same
         pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

       // pcl::io::savePLYFileASCII ("/home/julian/output2.ply", *cloud);

         //statistical removal filter
         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
         pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
         sor.setInputCloud (cloud);
         sor.setMeanK (50);
         sor.setStddevMulThresh (1.0);
         sor.filter (*cloud_filtered);

        // pcl::io::savePLYFileASCII ("/home/julian/output3.ply", *cloud_filtered);

        // voxel grid

//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_grid (new pcl::PointCloud<pcl::PointXYZ>());
//        pcl::VoxelGrid<pcl::PointXYZ> voxelObject;
//        voxelObject.setInputCloud (cloud);
//        voxelObject.setLeafSize (0.2f, 0.2f, 0.2f);
//        voxelObject.filter (*cloud_voxel_grid);




       // std::cout<<cloud_filtered->height<<std::endl;

	
        pcl::visualization::CloudViewer viewer ("Cluster viewer");
        viewer.showCloud (cloud_filtered);
        while (!viewer.wasStopped ())
        {
        }

return (0);
}

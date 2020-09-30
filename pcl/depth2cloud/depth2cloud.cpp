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

#define K_CX 319.5
#define K_CY 239.5
#define K_FX 525.0
#define K_FY 525.0
#define K_ROWS 480
#define K_COLS 640


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


        // convert depth image to pointcloud

        pcl::io::OrganizedConversion<pcl::PointXYZ,false>::convert(vec_image, dummy, false,cols, rows, focalLength,*cloud);

       // pcl::io::savePCDFileASCII ("/home/julian/output0437.pcd", *cloud);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate (Eigen::AngleAxisf (M_PI /*180°*/, Eigen::Vector3f::UnitX ()));

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
         // You can either apply transform_1 or transform_2; they are the same
         pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

         pcl::io::savePCDFileASCII ("/home/julian/output0436.pcd", *transformed_cloud);
	
        pcl::visualization::CloudViewer viewer ("Cluster viewer");
        viewer.showCloud (transformed_cloud);
        while (!viewer.wasStopped ())
        {
        }

return (0);
}


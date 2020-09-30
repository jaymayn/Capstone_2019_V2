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

void depth2cloud(cv::Mat *depth_image,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointXYZ point;

    int img_height = depth_image->rows;

    int img_width = depth_image->cols;

    const short unsigned * depth_data = reinterpret_cast<const short unsigned *>(&depth_image->data[0]);

    int row_step = depth_image->step /sizeof(const short unsigned);

    for (int v = 0; v<img_height; v++, depth_data += row_step)
{

        for (int u = 0; u<img_width; u++)
{

            unsigned int depth = depth_data[u];

            if (depth!=0)
{

                float depth_metric = (float)depth/1000.0f;

                point.y =  depth_metric *(K_CX-u) / K_FX;

                point.z =  depth_metric *(K_CY-v) / K_FY;

                point.x = depth_metric;

                cloud->push_back(point);

            }

        }

    }
}

int main (int argc, char** argv)
{

        cv::Mat * depth_image1 = new cv::Mat(cv::Mat::zeros(480,640, CV_16UC1));
        cv::Mat depth_image2;
        *depth_image1=cv::imread("/home/julian/box2_depth_image_raw_jpg/left0437.jpg",CV_LOAD_IMAGE_UNCHANGED);
        depth_image2=cv::imread("/home/julian/box2_depth_image_raw_jpg/left0438.jpg",CV_LOAD_IMAGE_UNCHANGED);


        pcl::PointCloud<pcl::PointXYZ> pc1;
        pcl::PointCloud<pcl::PointXYZ> pc2;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

         // You can either apply transform_1 or transform_2; they are the same

         // pcl::io::savePLYFileASCII ("/home/julian/output.ply", *cloud);




        depth2cloud(depth_image1,cloud);
      //  depth2cloud(depth_image2, pc2);

       // pcl::io::savePCDFileASCII ("/home/julian/output0433.pcd", *cloud);


       // pcl::PointCloud<pcl::PointXYZ>::Ptr mtrPtCloud (new pcl::PointCloud<pcl::PointXYZ> ());

//     pcl::PointCloud<pcl::PointXYZ> mtrPtCloud;

//     //NOTE - NEED TWO TRANSFORM BOTH CLOUDS SO THEY HAVE THE SAME ORIGIN

////        pcl::PointCloud<pcl::PointXYZ> recieved_transformed;
////        Eigen::Transform<cv::Scalar, 3, Eigen::Affine> recieved_transformation_mat(recieved.sensor_origin_ * recieved.sensor_orientation_);
////        pcl::transformPointCloud(recieved, pc1, recieved_transformation_mat);

////        pcl::PointCloud<pcl::PointXYZ> frame_transformed;
////        Eigen::Transform<cv::Scalar, 3, Eigen::Affine> frame_transformation_mat(frame.sensor_origin_ * frame.sensor_orientation_);
////        pcl::transformPointCloud(frame, pc2, frame_transformation_mat);


//        mtrPtCloud = pc1;
//        mtrPtCloud += pc2;






	
        pcl::visualization::CloudViewer viewer ("Cluster viewer");
        viewer.showCloud(cloud);
        while (!viewer.wasStopped ())
        {
        }

return (0);
}

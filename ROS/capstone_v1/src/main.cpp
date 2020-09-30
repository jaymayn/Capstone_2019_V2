#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <capstone_v1/depth2cloud.h>

#include <sstream>
#include <iostream>
#include <random>

#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/compression/organized_pointcloud_conversion.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

void planeSeg(PointCloudXYZ::Ptr cloud, PointCloudXYZ::Ptr cloud_inliers, PointCloudXYZ::Ptr cloud_outliers)
{
   

    // Segment the ground
    pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);

    pcl::PointIndices::Ptr 	inliers_plane (new pcl::PointIndices);

    PointCloudXYZ::Ptr cloud_plane (new PointCloudXYZ);

    // Make room for a plane equation (ax+by+cz+d=0)
    plane->values.resize (4);

    pcl::SACSegmentation<PointXYZ> seg;

        seg.setOptimizeCoefficients (true);			
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setDistanceThreshold (0.5f);
        seg.setInputCloud (cloud);
        seg.segment (*inliers_plane, *plane);

    // Extract inliers
        pcl::ExtractIndices<PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers_plane);
        extract.setNegative (false);			// Extract the inliers
        extract.filter (*cloud_inliers);		// cloud_inliers contains the plane

        // Extract outliers

        extract.setNegative (true);				// Extract the outliers
        extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

}

int main(int argc, char **argv)
{


// load depth image and convert to cv to ROS image using cv::bridge

cv::Mat depthImage;

depthImage = cv::imread("/home/julian/box2_depth_image_raw_jpg/left0437.jpg", CV_LOAD_IMAGE_UNCHANGED);

sensor_msgs::ImagePtr rosDepthImage = cv_bridge::CvImage(std_msgs::Header(), "16UC1", depthImage ).toImageMsg();

// initalise pointcloud and pass image + empty pointcloud to depth2cloud class

 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

 depth2cloud obj;

 obj.depthToPointCloud(rosDepthImage, cloud);

// downsample pointcloud using voxel grid

// apply segmentation to detect largest plane
////////////////////////////////////////////////////////////////////////////////////////

 PointCloudXYZ::Ptr cloud_inliers (new PointCloudXYZ)
                , cloud_outliers (new PointCloudXYZ);

 planeSeg(cloud, cloud_inliers, cloud_outliers);


// initailise cloud viewer options to view colored plane

// remove detected plane and repeat segmentation to extract 2nd largest plane

// project resulting plane to perpendicular view by transforming origin of cloud

//visualise

//	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

//	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_inliers_handler (cloud, 255, 20, 20); // Plane in RED
//	viewer.addPointCloud<PointXYZ> (cloud_inliers, cloud_inliers_handler, "cloud inliers");

//	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_outliers_handler (cloud, 200, 200, 200); // Everything else in GRAY
//	viewer.addPointCloud<PointXYZ> (cloud_outliers, cloud_outliers_handler, "cloud outliers");

        pcl::visualization::CloudViewer viewer ("PCL viewer");
        viewer.showCloud(cloud_outliers);
        while (!viewer.wasStopped ())
        {
		//viewer.spinOnce();
        }

  return 0;
}
















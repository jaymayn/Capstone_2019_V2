
#ifndef __Depth2Cloud_h_
#define __Depth2Cloud_h_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include<Eigen/Core>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>

#define K_CX 319.5
#define K_CY 239.5
#define K_FX 525.0
#define K_FY 525.0
#define K_ROWS 480
#define K_COLS 640

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::ModelCoefficients Plane3D;

class depth2cloud
{
public:


void depthToPointCloud(const sensor_msgs::ImageConstPtr depth_msg, PointCloudXYZ::Ptr pc)

{

    // Check that both image dimesions are non zero

    if ( (depth_msg->height == 0) || (depth_msg->width == 0) ) {

        ROS_ERROR("Depth image has invalid size");

        return;

    }

 

    int img_height = depth_msg->height;

    int img_width = depth_msg->width;

    pc->clear();

    pc->reserve(img_height*img_width);

 

    // Hardcoded pinhole camera parameters

    //TODO should be provided by configuration instead.

 

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) 
{

 

        pcl::PointXYZ point;

        const short unsigned * depth_data = reinterpret_cast<const short unsigned *>(&depth_msg->data[0]);

        int row_step = depth_msg->step /sizeof(const short unsigned);

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

                    pc->push_back(point);

                }

            }

        }

    } 

else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) 
{

        // Deal with metric depth image

        ROS_WARN("TYPE_32FC1 encoding not implemented in floor finder");

    } 
else 
{

        ROS_WARN_STREAM("Unknown image encoding in floor finder: " << depth_msg->encoding);

    }

    // Convert and copy the header

    pc->header = pcl_conversions::toPCL(depth_msg->header);

    //    pc_->header.frame_id = pc_->header.frame_id + "_floor";

    pc->header.frame_id = "/world";

}

 





};

#endif

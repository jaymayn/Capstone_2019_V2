// ros executable node with class
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#define K_CX 319.5
#define K_CY 239.5
#define K_FX 525.0
#define K_FY 525.0

class depth2cloud
{
public:
explicit depth2cloud(ros::NodeHandle& nh);  // Constructor


void publishPointCloud(const ros::TimerEvent&);

void depthCallback(const sensor_msgs::ImageConstPtr& msg);
void convertImageToPointCloud(sensor_msgs::ImageConstPtr& imageIn_);
void depthToPointCloud(sensor_msgs::ImageConstPtr& depth_msg);


protected:

ros::Subscriber sub_;
ros::Publisher pub_;

cv_bridge::CvImage imageOut_;

sensor_msgs::ImageConstPtr depth_msg_; 
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;


};

depth2cloud::depth2cloud(ros::NodeHandle& nh)
{
  sub_ = nh.subscribe("/box2/depth/image_raw", 1000, &depth2cloud::depthCallback, this);

  cv::namedWindow("window");


}

void depth2cloud::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::ImageConstPtr imageIn_;
    imageIn_ = msg;
    convertImageToPointCloud(imageIn_);

}
void depth2cloud::convertImageToPointCloud(sensor_msgs::ImageConstPtr& imageIn_)
{
	//convert image to cv bridge and cv mat and display in window

    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(imageIn_, "32FC1");
    cv::Mat depthImage = cv_ptr->image;

    cv::imshow("window",depthImage);
    cv::waitKey(3);
}

void depth2cloud::depthToPointCloud(sensor_msgs::ImageConstPtr& depth_msg)

{

    // Check that both image dimesions are non zero

    if ( (depth_msg->height == 0) || (depth_msg->width == 0) ) {

        ROS_ERROR("Depth image has invalid size");

        return;

    }



    int img_height = depth_msg->height;

    int img_width = depth_msg->width;

    pc_->clear();

    pc_->reserve(img_height*img_width);



    // Hardcoded pinhole camera parameters

    //TODO should be provided by configuration instead.



    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
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

                    pc_->push_back(point);

                }

            }

        }

    } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {

        // Deal with metric depth image

        ROS_WARN("TYPE_16UC1 encoding not implemented in floor finder");

    } else {

        ROS_WARN_STREAM("Unknown image encoding in floor finder: " << depth_msg->encoding);

    }

    // Convert and copy the header

    pc_->header = pcl_conversions::toPCL(depth_msg->header);

    //    pc_->header.frame_id = pc_->header.frame_id + "_floor";

    pc_->header.frame_id = "/world";

}

void depth2cloud::publishPointCloud(const ros::TimerEvent&)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_to_pointcloud");

  ros::NodeHandle nh{};

 // capstone_v2::depth2cloud depth2cloudNode(nh); <- if its within a namespace (best practise)
  depth2cloud depth2cloudNode(nh);

  
  ros::spin();

  return 0;
}

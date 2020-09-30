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
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <math.h>

#define K_CX 319.5
#define K_CY 239.5
#define K_FX 525.0
#define K_FY 525.0
#define K_ROWS 480
#define K_COLS 640

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class depthImagePreProcessing
{
public:
explicit depthImagePreProcessing(ros::NodeHandle& nh); 

void depthCallback(const sensor_msgs::ImageConstPtr& msg);
void showDepthImage(sensor_msgs::ImageConstPtr& imageIn_);
void depthToPointCloud(sensor_msgs::ImageConstPtr& depth_msg);


void pointCloudTransform(sensor_msgs::PointCloud2& cloudIn);
void pointCloudToDepthImage(PointCloudT::Ptr &cloudin);


protected:

ros::Subscriber sub_;
ros::Publisher pub_;
ros::Publisher pub2_;

cv_bridge::CvImage imageOut_;

sensor_msgs::ImageConstPtr depth_msg_; 

sensor_msgs::PointCloud2::ConstPtr cloud_; 

sensor_msgs::PointCloud2::ConstPtr processed_cloud_; 

sensor_msgs::ImageConstPtr processed_depth_msg_; 

int count;


};

depthImagePreProcessing::depthImagePreProcessing(ros::NodeHandle& nh)
{
  sub_ = nh.subscribe("/box2/depth/image_raw", 1000, &depthImagePreProcessing::depthCallback, this);

  pub_ = nh.advertise<sensor_msgs::PointCloud2>("/box2/pointcloud", 1000);

  pub2_ = nh.advertise<sensor_msgs::Image>("/box2/depth/plane_converted/image_raw", 1000);


  count = 0;
  cv::namedWindow("window");


}

void depthImagePreProcessing::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::ImageConstPtr imageIn_;
    imageIn_ = msg;
    //showDepthImage(imageIn_);
    depthToPointCloud(imageIn_);

}
void depthImagePreProcessing::showDepthImage(sensor_msgs::ImageConstPtr& imageIn_)
{
	//convert image to cv bridge and cv mat and display in window

    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(imageIn_, "32FC1");
    cv::Mat depthImage = cv_ptr->image;

    cv::imshow("window",depthImage);
    cv::waitKey(3);
}

void depthImagePreProcessing::depthToPointCloud(sensor_msgs::ImageConstPtr& depth_msg)
{
   
 cv::Mat cv_frame16 = cv_bridge::toCvCopy(depth_msg,sensor_msgs::image_encodings::TYPE_16UC1)->image;

 sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, cv_frame16).toImageMsg();
    // Check that both image dimesions are non zero

    if ( (depth_msg->height == 0) || (depth_msg->width == 0) ) 
      {

        ROS_ERROR("Depth image has invalid size");

        return;
      }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ (new pcl::PointCloud<pcl::PointXYZ>);

    int img_height = depth_msg->height;

    int img_width = depth_msg->width;

    pc_->clear();

    pc_->reserve(img_height*img_width);



    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        pcl::PointXYZ point;

        const short unsigned * depth_data = reinterpret_cast<const short unsigned *>(&msg->data[0]);

        int row_step = msg->step /sizeof(const short unsigned);

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

    }
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        // Deal with 8 bit 16uc1

        ROS_WARN("TYPE_16UC1 encoding not implemented in floor finder");

    }
    else
    {

        ROS_WARN_STREAM("Unknown image encoding in floor finder: " << depth_msg->encoding);

    }


    // Convert and copy the header

    pc_->header = pcl_conversions::toPCL(depth_msg->header);

    pc_->header.frame_id = "/world";

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*pc_, output);

    //pcl::toROSMsg(*pc_, cloud_);

    pointCloudTransform(output);



}




void depthImagePreProcessing::pointCloudTransform(sensor_msgs::PointCloud2 &cloudIn)
{

//downsample - leaf size of 1cm

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> sor;
//sor.setInputCloud(cloud);
//sor.setLeafSize(0.01f,0.01f,0.01f);
//sor.filter(*cloud_filtered);

    PointCloudT::Ptr cloud (new PointCloudT);
    PointCloudT::Ptr cloud_inliers (new PointCloudT);
    PointCloudT::Ptr cloud_outliers (new PointCloudT);

        pcl::fromROSMsg(cloudIn, *cloud);

	// Segment the ground
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr 		inliers_plane (new pcl::PointIndices);
	PointCloudT::Ptr 			cloud_plane (new PointCloudT);

	// Make room for a plane equation (ax+by+cz+d=0)
	plane->values.resize (4);

	pcl::SACSegmentation<PointT> seg;				// Create the segmentation object
	seg.setOptimizeCoefficients (true);				// Optional
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setModelType (pcl::SACMODEL_PLANE);
    //seg.setDistanceThreshold (0.005f);
        seg.setDistanceThreshold (0.17f);
	seg.setInputCloud (cloud);
	seg.segment (*inliers_plane, *plane);

//    if (inliers_plane->indices.size () == 0)
//    {
//		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
//		return (-1);
//	}

	// Extract inliers
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);			// Extract the inliers
	extract.filter (*cloud_inliers);		// cloud_inliers contains the plane

	// Extract outliers
	extract.setNegative (true);				// Extract the outliers
	extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane


    //determine rotation and translation from origin (yz plane) to extracted plane

    Eigen::Vector3f plane_normal_vector, yz_plane_normal_vector, rotation_vector, translation_vector, transformVector;

          plane_normal_vector[0] = plane->values[0];
          plane_normal_vector[1] = plane->values[1];
          plane_normal_vector[2] = plane->values[2];

          std::cout << plane_normal_vector << std::endl;

          yz_plane_normal_vector[0] = 0.0;
          yz_plane_normal_vector[1] = 0.0;
          yz_plane_normal_vector[2] = 1.0;

          std::cout << yz_plane_normal_vector << std::endl;

    	rotation_vector = plane_normal_vector.cross(yz_plane_normal_vector);

  	float length = std::sqrt(rotation_vector[0]*rotation_vector[0]+rotation_vector[1]*rotation_vector[1]+rotation_vector[2]*rotation_vector[2]);

  	transformVector(0) = rotation_vector[0] / length;
  	transformVector(1) = rotation_vector[1] / length;
  	transformVector(2) = rotation_vector[2] / length; 

 	Eigen::Affine3f transformRotationOfModel = Eigen::Affine3f::Identity();

  	float theta1 = acos(plane_normal_vector[0]*yz_plane_normal_vector[0]+plane_normal_vector[1]*yz_plane_normal_vector[1]+plane_normal_vector[2]*yz_plane_normal_vector[2]);


  	transformRotationOfModel.rotate (Eigen::AngleAxisf(theta1, transformVector));


       PointCloudT::Ptr	vector_transformed_cloud (new PointCloudT());
       pcl::transformPointCloud (*cloud_inliers, *vector_transformed_cloud, transformRotationOfModel);

       sensor_msgs::PointCloud2 processed_cloud_;

        pcl::toROSMsg(*vector_transformed_cloud, processed_cloud_);

    pub_.publish (processed_cloud_);

    pointCloudToDepthImage(vector_transformed_cloud);






}


void depthImagePreProcessing::pointCloudToDepthImage(PointCloudT::Ptr &cloudin)
{

    cv::Mat inputMat = cv::Mat::zeros(K_ROWS, K_COLS, CV_32F );

    for(int r = 0; r < cloudin->points.size(); r++)
	{

        pcl::PointXYZ pt = cloudin->points[r];

        float Di = pt.z*1000;

        if(Di != 0)
           {

            int u = roundf(K_CX - pt.x*K_FX/pt.z);

            int v = roundf(K_CY - pt.y*K_FY/pt.z);

            if(u < K_COLS && u > 0 && v < K_ROWS && v > 0)
		{

                inputMat.at<float>(v,u) = Di;

            	}

           }

    	}

  //  processed_depth_msg_=inputMat;

    
    cv::Mat img; // << image MUST be contained here
    sensor_msgs::Image published_depth_img;

cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg; // >> message to be sent

std_msgs::Header header; // empty header
header.seq = count; // user defined counter
header.stamp = ros::Time::now(); // time

img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, inputMat);

img_bridge.toImageMsg(published_depth_img); // from cv_bridge to sensor_msgs::Image

pub2_.publish(published_depth_img); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_to_pointcloud");

  ros::NodeHandle nh{};

  depthImagePreProcessing depthImagePreProcNode(nh);
  
  ros::spin();

  return 0;
}


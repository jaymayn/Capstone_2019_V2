#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <Eigen/Dense>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



int
main (int argc, char *argv[])
{

	PointCloudT::Ptr	cloud (new PointCloudT),
						cloud_inliers (new PointCloudT),
						cloud_outliers (new PointCloudT);

	// Load point cloud
    if (pcl::io::loadPLYFile ("/home/julian/Desktop/depthpcl_1.ply", *cloud) < 0)
    {
		PCL_ERROR ("Could not load PCD file !\n");
		return (-1);
	}

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
        seg.setDistanceThreshold (0.1f);
	seg.setInputCloud (cloud);
	seg.segment (*inliers_plane, *plane);

	if (inliers_plane->indices.size () == 0) {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
		return (-1);
	}

	// Extract inliers
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);			// Extract the inliers
	extract.filter (*cloud_inliers);		// cloud_inliers contains the plane

	// Extract outliers
	//extract.setInputCloud (cloud);		// Already done line 50
	//extract.setIndices (inliers);			// Already done line 51
	extract.setNegative (true);				// Extract the outliers
	extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

    // Segment the ground
    pcl::ModelCoefficients::Ptr plane2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr 		inliers_plane2 (new pcl::PointIndices);
    PointCloudT::Ptr 			cloud_plane2 (new PointCloudT);

plane2->values.resize (4);

    pcl::SACSegmentation<PointT> seg2;				// Create the segmentation object
    seg2.setOptimizeCoefficients (true);				// Optional
    seg2.setMethodType (pcl::SAC_RANSAC);
    seg2.setModelType (pcl::SACMODEL_PLANE);
    //seg.setDistanceThreshold (0.005f);
        seg2.setDistanceThreshold (0.26f);
    seg2.setInputCloud (cloud_outliers);
    seg2.segment (*inliers_plane2, *plane2);

    if (inliers_plane2->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
        return (-1);
    }

    PointCloudT::Ptr	cloud_inliers2 (new PointCloudT),
                        cloud_outliers2 (new PointCloudT);

    // Extract inliers
    pcl::ExtractIndices<PointT> extract2;
    extract2.setInputCloud (cloud_outliers);
    extract2.setIndices (inliers_plane2);
    extract2.setNegative (false);			// Extract the inliers
    extract2.filter (*cloud_inliers2);		// cloud_inliers contains the plane

    // Extract outliers
    //extract.setInputCloud (cloud);		// Already done line 50
    //extract.setIndices (inliers);			// Already done line 51
    extract2.setNegative (true);				// Extract the outliers
    extract2.filter (*cloud_outliers2);		// cloud_outliers contains everything but the plane


	printf ("Plane segmentation equation [ax+by+cz+d]=0: [%3.4f | %3.4f | %3.4f | %3.4f]     \t\n", 
			plane->values[0], plane->values[1], plane->values[2] , plane->values[3]);

	// Visualization
	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler (cloud, 255, 20, 20); // Plane in RED
	viewer.addPointCloud (cloud_inliers, cloud_inliers_handler, "cloud inliers");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler2 (cloud, 20, 255, 20); // Everything else in GRAY
    viewer.addPointCloud (cloud_inliers2, cloud_inliers_handler2, "cloud inliers2");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_outliers_handler (cloud, 200, 200, 200); // Everything else in GRAY
    viewer.addPointCloud (cloud_outliers2, cloud_outliers_handler, "cloud outliers");

	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}
	return (0);
}
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <math.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



int
main (int argc, char *argv[])
{

	PointCloudT::Ptr	cloud (new PointCloudT),
						cloud_inliers (new PointCloudT),
						cloud_outliers (new PointCloudT);

	// Load point cloud
    if (pcl::io::loadPLYFile ("/home/julian/box2-450.ply", *cloud) < 0)
    {
        PCL_ERROR ("Could not load PCD file !\n");
        return (-1);
    }

//downsample - leaf size of 1cm

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> sor;
//sor.setInputCloud(cloud);
//sor.setLeafSize(0.01f,0.01f,0.01f);
//sor.filter(*cloud_filtered);

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

    if (inliers_plane->indices.size () == 0)
    {
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
	extract.setNegative (true);				// Extract the outliers
	extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

	printf ("Plane segmentation equation [ax+by+cz+d]=0: [%3.4f | %3.4f | %3.4f | %3.4f]     \t\n", 
			plane->values[0], plane->values[1], plane->values[2] , plane->values[3]);

    pcl::PointXYZRGBA minPt, maxPt;
    pcl::getMinMax3D (*cloud_inliers, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    

    double desired_focal_centre_X = (minPt.x + maxPt.x) / 2;
    double desired_focal_centre_Y = (minPt.y + maxPt.y) / 2;
    double desired_focal_centre_Z = (minPt.z + maxPt.z) / 2;





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

    translation_vector[0] = plane_normal_vector[0] - yz_plane_normal_vector[0];
    translation_vector[1] = plane_normal_vector[1] - yz_plane_normal_vector[1];
    translation_vector[2] = plane_normal_vector[2] - yz_plane_normal_vector[2];




  float length = std::sqrt(rotation_vector[0]*rotation_vector[0]+rotation_vector[1]*rotation_vector[1]+rotation_vector[2]*rotation_vector[2]);

  transformVector(0) = rotation_vector[0] / length;
  transformVector(1) = rotation_vector[1] / length;
  transformVector(2) = rotation_vector[2] / length; 

 Eigen::Affine3f transformRotationOfModel = Eigen::Affine3f::Identity();

 float theta1 = acos(plane_normal_vector[0]*yz_plane_normal_vector[0]+plane_normal_vector[1]*yz_plane_normal_vector[1]+plane_normal_vector[2]*yz_plane_normal_vector[2]);


 transformRotationOfModel.rotate (Eigen::AngleAxisf(theta1, transformVector));
 transformRotationOfModel.translation() << desired_focal_centre_X - 1.0, desired_focal_centre_Y, 0.0;

 //transformRotationOfModel.translation() << -0.9, desired_focal_centre_Y, -desired_focal_centre_Z;









//      // Executing the transformation
//       PointCloudT::Ptr manually_transformed_cloud (new PointCloudT ());
//       pcl::transformPointCloud (*cloud_inliers, *manually_transformed_cloud, transform_2);

       PointCloudT::Ptr	vector_transformed_cloud (new PointCloudT());
       pcl::transformPointCloud (*cloud_inliers, *vector_transformed_cloud, transformRotationOfModel);



	// Visualization
	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");


    viewer.addCoordinateSystem(1.0,"origin", 0);
   viewer.addCoordinateSystem(1.0,desired_focal_centre_X, -desired_focal_centre_Y, desired_focal_centre_Z, 0);


        pcl::ModelCoefficients plane_coeff;
        plane_coeff.values.resize (4);
        plane_coeff.values[0] = plane->values[0];
        plane_coeff.values[1] = plane->values[1];
        plane_coeff.values[2] = plane->values[2];
        plane_coeff.values[3] = plane->values[3];

   // viewer.addPlane(plane_coeff,"p",0); // plane equation

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler (cloud, 200, 200, 200); // transformed Plane in RED
 //   viewer.addPointCloud (manually_transformed_cloud, cloud_inliers_handler, "cloud inliers");

  //  viewer.addPointCloud (cloud_inliers, cloud_inliers_handler, "cloud inliers");


    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler2 (cloud, 20, 255, 20); // Plane in green
    viewer.addPointCloud (vector_transformed_cloud, cloud_inliers_handler2, "cloud inliers2");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_outliers_handler (cloud, 200, 200, 200); // Everything else in GRAY
    //viewer.addPointCloud (cloud_outliers, cloud_outliers_handler, "cloud outliers");

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

	return (0);
}

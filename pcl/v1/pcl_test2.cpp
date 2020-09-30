#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/opencv.hpp>
#include <pcl/compression/organized_pointcloud_conversion.h>



void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr &downsampled_output)
{
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox_grid.setInputCloud(cloud);
    vox_grid.filter(*downsampled_output);
}


void compute_surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float normal_radius, pcl::PointCloud<pcl::Normal>::Ptr &normals_output)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

    norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));

    norm_est.setRadiusSearch(normal_radius);

    norm_est.setInputCloud(cloud);

    norm_est.compute(*normals_output);

}


void visualize_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr normal_points, const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud(cloud, "points");
    viz.addPointCloud(normal_points, "normals_points");
    viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (normal_points, normals, 1, 0.01);
    viz.spin();
}











int main (int argc, char** argv)
{
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ds (new pcl::PointCloud<pcl::PointXYZ>);

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

//  if (pcl::io::loadPLYFile<pcl::PointXYZRGB> ("/home/julian/output2.ply", *cloud) == -1) //* load the file
//  {
//    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//    return (-1);
//  }

  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);


  //voxelgrid filter

  const float leaf_size = 0.01;

  downsample(cloud, leaf_size, ds);

  //normal estimation

  const float normal_radius = 0.03;
  compute_surface_normals(ds, normal_radius, normals);

  //visualise

  //visualize_normals(cloud, ds, normals);







//    cloud->width = 640;
//    cloud->height = 480;


//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//     viewer.showCloud (ds);
//     while (!viewer.wasStopped ())
//       {

//     }

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (ds);
  seg.segment (*inliers, *coefficients);



  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
      std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                 << cloud->points[inliers->indices[i]].y << " "
                                                 << cloud->points[inliers->indices[i]].z << std::endl;


pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

 viewer.addPointCloud(ds, "points");

viewer.addPlane(*coefficients, "plane_1", 0);
viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1 /*R,G,B*/, "plane_1", 0);
viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane_1", 0);
viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "plane_1", 0);

viewer.addCoordinateSystem (0.5, "axis", 0);
viewer.setBackgroundColor (0.05, 0.05, 0.05, 0);
//viewer.setPosition (800, 400);

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }


   return (0);
}

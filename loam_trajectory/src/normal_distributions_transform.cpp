//pcl::normal_distributions_transform的例程

#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;


int main (int argc, char** argv)
{
  ros::init(argc,argv,"normal_distributions_transform");

  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/wp/vr_ws/ndt/room_scan1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/wp/vr_ws/ndt/room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);//(1.79387, 0.720047, 0)
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);



  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  std::cout << ndt.getFinalTransformation() << std::endl;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> output_cloud_(output_cloud->points.size(),1);

  Eigen::Matrix<float,4,4> transform = ndt.getFinalTransformation();
  Eigen::Matrix<float,3,3> rotation;
  Eigen::Matrix<float,3,1> T;
  rotation = transform.block<3,3> (0,0);
  T = transform.block<3,1> (0,3);
  Eigen::Matrix<float,3,3> rotation_ = rotation.transpose();
  Eigen::Matrix<float,3,1> T_ = -T;
  Eigen::Matrix<float,4,4> transform_;
  transform_.setZero();
  transform_.block<3,3> (0,0) = rotation_;
  //transform_.block<3,1> (0,3) = T_;
  transform_ (3,3) = 1;
  std::cout << transform_ <<std::endl;

  Eigen::Matrix<float,3,1> point;
  Eigen::Matrix<float,3,1> point_;


  for (int i=0; i< output_cloud->points.size(); i++)
  {
      point(0) = output_cloud->points[i].x;
      point(1) = output_cloud->points[i].y;
      point(2) = output_cloud->points[i].z;
      //point(3) = 1;

      point = point + T_;
      point = rotation_ * point;

      point_(0) = point(0);
      point_(1) = point(1);
      point_(2) = point(2);
//      *output_cloud_ += *point_;
      output_cloud_.points[i].x = point(0);
      output_cloud_.points[i].y = point(1);
      output_cloud_.points[i].z = point(2);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
  *output_cloud_1 = output_cloud_;
  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("/home/wp/vr_ws/ndt/room_scan2_transformed.pcd", *output_cloud);
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color (input_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (input_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color_ (output_cloud_1, 0, 0, 255);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud_1, output_color_, "output_ cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_ cloud");

  // Starting visualizer
  //viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}

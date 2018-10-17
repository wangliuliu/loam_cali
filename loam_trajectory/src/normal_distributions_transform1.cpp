//pcl::normal_distributions_transform的例程

#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

bool first_ndt = true;
#define PI  3.1415926


int main (int argc, char** argv)
{
  ros::init(argc,argv,"normal_distributions_transform1");

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);//地图

  char *Path= new char[100];
  sprintf(Path,"/home/wp/vr_ws/map_campus_release1/map_campus%d.pcd",100);
  string nPath1(Path);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);//目标点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ (new pcl::PointCloud<pcl::PointXYZ>);//转换到初始坐标系的目标点云
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (nPath1, *target_cloud) == -1)//ndt/room_scan1.pcd
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }//读取第一帧数据

  *map_cloud += *target_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);//输入点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ (new pcl::PointCloud<pcl::PointXYZ>);//转换到初始坐标系的输入点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr odometry_cloud (new pcl::PointCloud<pcl::PointXYZ>);//里程计
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);//输出点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ (new pcl::PointCloud<pcl::PointXYZ>);//最终输出点云，作为下一次的目标点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);//用于存储输入点云滤波的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ (new pcl::PointCloud<pcl::PointXYZ>);//用于存储地图滤波的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud (new pcl::PointCloud<pcl::PointXYZ>);//用于存储地图滤波的点云
  Eigen::Matrix4f init_guess;
  Eigen::Matrix4f last_guess;
  Eigen::Matrix<float,4,4> transform;
  Eigen::Matrix<float,3,3> R;
  Eigen::Matrix<float,3,1> T;

  sprintf(Path,"/home/wp/vr_ws/odometry_campus_release1/odometry_campus%d.pcd",100);
  string nPath4(Path);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (nPath4, *odometry_cloud) == -1)//ndt/room_scan2.pcd
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  float roll_old = odometry_cloud->points[1].x / 180 * PI;
  float pitch_old = odometry_cloud->points[1].y / 180 * PI;
  float yaw_old = odometry_cloud->points[1].z / 180 * PI;
  float x_old = odometry_cloud->points[0].x;
  float y_old = odometry_cloud->points[0].y;
  float z_old = odometry_cloud->points[0].z;

  float roll_first = roll_old;
  float pitch_first = pitch_old;
  float yaw_first = yaw_old;
  float x_first = x_old;
  float y_first = y_old;
  float z_first = z_old;

  for (int i=101; i<2395 ; i++)//2395
  {
      std::cout<< i <<std::endl;

      Eigen::Matrix<float,3,1> point;
      Eigen::Matrix<float,4,1> point_;

      Eigen::Matrix<float,3,3> target_rotationx;
      Eigen::Matrix<float,3,3> target_rotationy;
      Eigen::Matrix<float,3,3> target_rotationz;
      target_rotationx << 1,0,0,0,cos(-roll_old),sin(-roll_old),0,-sin(-roll_old),cos(-roll_old);
      target_rotationy << cos(-pitch_old),0,-sin(-pitch_old),0,1,0,sin(-pitch_old),0,cos(-pitch_old);
      target_rotationz << cos(-yaw_old),sin(-yaw_old),0,-sin(-yaw_old),cos(-pitch_old),0,0,0,1;

      Eigen::Matrix<float,3,3> target_rotation = target_rotationz * target_rotationy * target_rotationx;
      Eigen::Matrix<float,3,1> target_translation;
      target_translation << -x_old, -y_old, -z_old;

      *target_cloud_ = *target_cloud;
      for (int j=0; j< target_cloud->points.size(); j++)
      {
          point(0) = target_cloud->points[j].x;
          point(1) = target_cloud->points[j].y;
          point(2) = target_cloud->points[j].z;
          point = point + target_translation;
          point = target_rotation * point;

          target_cloud_->points[j].x = point(0);
          target_cloud_->points[j].y = point(1);
          target_cloud_->points[j].z = point(2);
      }

      sprintf(Path,"/home/wp/vr_ws/map_campus_release1/map_campus%d.pcd",i);
      string nPath2(Path);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (nPath2, *input_cloud) == -1)//ndt/room_scan2.pcd
      {
        PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
        return (-1);
      }
      sprintf(Path,"/home/wp/vr_ws/odometry_campus_release1/odometry_campus%d.pcd",i);
      string nPath3(Path);
      if (pcl::io::loadPCDFile<pcl::PointXYZ> (nPath3, *odometry_cloud) == -1)//ndt/room_scan2.pcd
      {
        PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
        return (-1);
      }
      double roll = odometry_cloud->points[1].x / 180 * PI;
      double pitch = odometry_cloud->points[1].y / 180 * PI;
      double yaw = odometry_cloud->points[1].z / 180 * PI;
      Eigen::Matrix<float,3,3> input_rotationx;
      Eigen::Matrix<float,3,3> input_rotationy;
      Eigen::Matrix<float,3,3> input_rotationz;
      input_rotationx << 1,0,0,0,cos(-roll),sin(-roll),0,-sin(-roll),cos(-roll);
      input_rotationy << cos(-pitch),0,-sin(-pitch),0,1,0,sin(-pitch),0,cos(-pitch);
      input_rotationz << cos(-yaw),sin(-yaw),0,-sin(-yaw),cos(-pitch),0,0,0,1;

      Eigen::Matrix<float,3,3> input_rotation = input_rotationz * input_rotationy * input_rotationx;
      Eigen::Matrix<float,3,1> input_translation;
      input_translation << -odometry_cloud->points[0].x, -odometry_cloud->points[0].y, -odometry_cloud->points[0].z;

      *input_cloud_ = *input_cloud;
      for (int j=0; j< input_cloud->points.size(); j++)
      {
          point(0) = input_cloud->points[j].x;
          point(1) = input_cloud->points[j].y;
          point(2) = input_cloud->points[j].z;
          point = point + input_translation;
          point = input_rotation * point;

          input_cloud_->points[j].x = point(0);
          input_cloud_->points[j].y = point(1);
          input_cloud_->points[j].z = point(2);
      }

      pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
      approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
      approximate_voxel_filter.setInputCloud (input_cloud_);
      approximate_voxel_filter.filter (*filtered_cloud);
      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
      ndt.setTransformationEpsilon (0.01);
      ndt.setStepSize (0.2);
      ndt.setResolution (10.0);
      ndt.setMaximumIterations (35);
      ndt.setInputSource (filtered_cloud);
      ndt.setInputTarget (target_cloud_);

      if (first_ndt)
      {
//          Eigen::AngleAxisf init_rotationx (0,Eigen::Vector3f::UnitX());//0.6931
//          Eigen::AngleAxisf init_rotationy (0,Eigen::Vector3f::UnitY());
//          Eigen::AngleAxisf init_rotationz (0,Eigen::Vector3f::UnitZ());
//          Eigen::Translation3f init_translation (0, 0, 0);
//          init_guess = (init_translation * init_rotationx * init_rotationy * init_rotationz).matrix ();
//          last_guess << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
//          init_guess.block<3,3> (0,0) = input_rotation.transpose() * target_rotation;
//          init_guess.block<3,1> (0,3) = target_translation - input_translation;
//          init_guess(3,3) = 1;
//          init_guess(3,0) = 0;
//          init_guess(3,1) = 0;
//          init_guess(3,2) = 0;
          init_guess.setZero();
          init_guess(0,0) = 1;
          init_guess(1,1) = 1;
          init_guess(2,2) = 1;
          init_guess(3,3) = 1;
          std::cout <<"init_guess=" <<init_guess << std::endl;
          first_ndt = false;
      }
      else
      {
          init_guess = transform;
          std::cout <<"init_guess=" <<init_guess << std::endl;
      }

      ndt.align (*output_cloud, init_guess);
      pcl::transformPointCloud (*input_cloud_, *output_cloud, ndt.getFinalTransformation ());
      transform = ndt.getFinalTransformation();
      std::cout<< "transform=" << transform <<std::endl;

      Eigen::Matrix<float,3,3> output_rotationx;
      Eigen::Matrix<float,3,3> output_rotationy;
      Eigen::Matrix<float,3,3> output_rotationz;
      output_rotationx << 1,0,0,0,cos(roll_old),sin(roll_old),0,-sin(roll_old),cos(roll_old);
      output_rotationy << cos(pitch_old),0,-sin(pitch_old),0,1,0,sin(pitch_old),0,cos(pitch_old);
      output_rotationz << cos(yaw_old),sin(yaw_old),0,-sin(yaw_old),cos(pitch_old),0,0,0,1;
      Eigen::Matrix<float,3,3> output_rotation = output_rotationx * output_rotationy * output_rotationz;
      Eigen::Matrix<float,3,1> output_translation;
      Eigen::Matrix<float,3,3> output_rotation_;
      Eigen::Matrix<float,3,1> output_translation_;
      output_translation << x_old, y_old, z_old;

      *output_cloud_ = *output_cloud;
      for (int j=0; j< output_cloud->points.size(); j++)
      {
          point(0) = output_cloud->points[j].x;
          point(1) = output_cloud->points[j].y;
          point(2) = output_cloud->points[j].z;

          point = output_rotation * point;
          point = point + output_translation;

//          point_(0) = point(0);
//          point_(1) = point(1);
//          point_(2) = point(2);
//          point_(3) = 1;

//          point_ = last_guess * point_;

//          output_cloud_->points[j].x = point_(0);
//          output_cloud_->points[j].y = point_(1);
//          output_cloud_->points[j].z = point_(2);
          output_cloud_->points[j].x = point(0);
          output_cloud_->points[j].y = point(1);
          output_cloud_->points[j].z = point(2);
      }

      output_rotation_ = output_rotation * transform.block<3,3> (0,0);
      output_translation_ = output_translation + transform.block<3,1> (0,3);

      roll_old = atan2(output_rotation_(1,2),output_rotation_(2,2));
      pitch_old = -asin(output_rotation_(0,2));
      yaw_old = atan2(output_rotation_(0,1),output_rotation_(0,0));
      x_old = output_translation_(0);
      y_old = output_translation_(1);
      z_old = output_translation_(2);
//      roll_old = roll;
//      pitch_old = pitch;
//      yaw_old =yaw;
//      x_old = odometry_cloud->points[0].x;
//      y_old = odometry_cloud->points[0].y;
//      z_old = odometry_cloud->points[0].z;
      //std::cout<<"r="<<roll_old<<";p="<<pitch_old<<";y="<<yaw_old<<";x="<<x_old<<";y="<<y_old<<";z="<<z_old<<std::endl;

      *map_cloud += *output_cloud_;

      pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter_;
      approximate_voxel_filter_.setLeafSize (0.2, 0.2, 0.2);
      approximate_voxel_filter_.setInputCloud (map_cloud);
      approximate_voxel_filter_.filter (*filtered_cloud_);

      map_cloud->clear();
      *map_cloud = *filtered_cloud_;
      *last_cloud = *target_cloud;
      target_cloud->clear();
      target_cloud_->clear();
      *target_cloud = *input_cloud;//input_cloud

      input_cloud->clear();
      input_cloud_->clear();
      output_cloud->clear();
      output_cloud_->clear();
      filtered_cloud->clear();
      filtered_cloud_->clear();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr down_size_target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterMap;
  downSizeFilterMap.setLeafSize(1.0f, 1.0f, 1.0f);
  downSizeFilterMap.setInputCloud(map_cloud);
  ROS_INFO("size1=%d\n",map_cloud->points.size());
  downSizeFilterMap.filter(*down_size_target_cloud);
  ROS_INFO("size2=%d\n",down_size_target_cloud->points.size());
  map_cloud->clear();
  *map_cloud = *down_size_target_cloud;
  ROS_INFO("size3=%d\n",map_cloud->points.size());
  pcl::io::savePCDFileASCII ("/home/wp/vr_ws/map_campus_release/map_campus_ndt.pcd", *map_cloud);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> map_color (map_cloud, 255, 255, 255);
  viewer_final->addPointCloud<pcl::PointXYZ> (map_cloud, map_color, "map cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "map cloud");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color (target_cloud, 255, 0, 0);
//  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color_ (target_cloud_, 100, 0, 0);
//  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud_, target_color_, "target cloud _");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud _");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color (input_cloud, 0, 255, 0);
//  viewer_final->addPointCloud<pcl::PointXYZ> (input_cloud, input_color, "input cloud");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "input cloud");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color_ (input_cloud_, 0, 100, 0);
//  viewer_final->addPointCloud<pcl::PointXYZ> (input_cloud_, input_color_, "input cloud _");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "input cloud _");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color (output_cloud, 0, 0, 255);
//  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "output cloud");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color_ (output_cloud_, 0, 0, 100);
//  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud_, output_color_, "output cloud _");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "output cloud _");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_color (filtered_cloud, 0, 255, 0);
//  viewer_final->addPointCloud<pcl::PointXYZ> (filtered_cloud, filtered_color, "filtered_cloud");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered_cloud");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_color_ (filtered_cloud_, 0, 100, 0);
//  viewer_final->addPointCloud<pcl::PointXYZ> (filtered_cloud_, filtered_color_, "filtered_cloud _");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered_cloud _");

//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> last_color (last_cloud, 100, 100, 100);
//  viewer_final->addPointCloud<pcl::PointXYZ> (last_cloud, last_color, "last cloud");
//  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "last cloud");

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

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include <nav_msgs/OccupancyGrid.h>

#include <eigen3/Eigen/Dense>

using namespace std;

float transformSum[6] = {0};
float transformSum2[6] = {0};

//wzy
Eigen::Quaterniond lastRotat(1,0,0,0);
Eigen::Vector3d lastTrans(0,0,0);
nav_msgs::Odometry odo_msg;


pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudRegistered2(new pcl::PointCloud<pcl::PointXYZI>());

int countss;
int cloudNum=0;
int odometryNum=0;

double roll, pitch, yaw;

ros::Publisher *pubLaserCloudSurround2Pointer=NULL;
ros::Publisher *pubLaserCloudRegistered2Pointer=NULL;
ros::Publisher *pubLaserOdometry2Pointer = NULL;

ros::Publisher *pubpath=NULL;
ros::Publisher *pubMap = NULL;

nav_msgs::OccupancyGrid map_2d;

tf::TransformBroadcaster *tfBroadcaster2Pointer = NULL;
tf::TransformBroadcaster *tfBroadcaster2Pointer1 = NULL;
tf::TransformBroadcaster *tfBroadcaster2Pointer2 = NULL;
nav_msgs::Odometry laserOdometry2;
nav_msgs::Path trajectory;

tf::StampedTransform laserOdometryTrans2;
tf::StampedTransform world_imu;
tf::StampedTransform vel_imu;

void laserCloudSurroundHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurround)
{
    double timeLaserCloudSurround = laserCloudSurround->header.stamp.toSec();

    laserCloudSurround2->clear();
    pcl::fromROSMsg(*laserCloudSurround, *laserCloudSurround2);

//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());
//    pcl::PassThrough<pcl::PointXYZI> pass;
//    pass.setInputCloud(laserCloudSurround2);
//    pass.setFilterFieldName("y");
//    pass.setFilterLimits(0,0.1);
//    pass.filter(*cloud_filtered);//注释的为滤除一定高度的点云，即主要用于滤除房顶

    for(int i=0;i<laserCloudSurround2->points.size();i++)
    {
        float x,y,z;
        x=laserCloudSurround2->points[i].x;
        y=laserCloudSurround2->points[i].y;
        z=laserCloudSurround2->points[i].z;
        laserCloudSurround2->points[i].x=z;
        laserCloudSurround2->points[i].y=x;
        laserCloudSurround2->points[i].z=y;
    }

//    for(int i=0;i<cloud_filtered->points.size();i++)
//    {
//        float x,y,z;
//        x=cloud_filtered->points[i].x;
//        y=cloud_filtered->points[i].y;
//        z=cloud_filtered->points[i].z;
//        cloud_filtered->points[i].x=z;
//        cloud_filtered->points[i].y=x;
//        cloud_filtered->points[i].z=y;
//        int mx = (cloud_filtered->points[i].x - map_2d.info.origin.position.x) / map_2d.info.resolution;
//        int my = (cloud_filtered->points[i].y - map_2d.info.origin.position.y) / map_2d.info.resolution;
//        if(mx<0 || mx>map_2d.info.width || my<0 || my>map_2d.info.height) continue;
//        int index = my*map_2d.info.height + mx;
//        map_2d.data[index] = 100;
//    }

    sensor_msgs::PointCloud2 laserCloudSurround3;
    //pcl::toROSMsg(*cloud_filtered, laserCloudSurround3);
    pcl::toROSMsg(*laserCloudSurround2, laserCloudSurround3);
    laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserCloudSurround);
    laserCloudSurround3.header.frame_id = "/camera_init";

    pubLaserCloudSurround2Pointer->publish(laserCloudSurround3);
//    map_2d.header.stamp = ros::Time().fromSec(timeLaserCloudSurround);
//    pubMap->publish(map_2d);
}

void CloudRegisteredHandler(const sensor_msgs::PointCloud2ConstPtr& CloudRegistered)
{
    double timeCloudRegistered = CloudRegistered->header.stamp.toSec();

    laserCloudRegistered2->clear();
    pcl::fromROSMsg(*CloudRegistered, *laserCloudRegistered2);

    for(int i=0;i<laserCloudRegistered2->points.size();i++)
    {
        float x,y,z;
        x=laserCloudRegistered2->points[i].x;
        y=laserCloudRegistered2->points[i].y;
        z=laserCloudRegistered2->points[i].z;
        laserCloudRegistered2->points[i].x=z;
        laserCloudRegistered2->points[i].y=x;
        laserCloudRegistered2->points[i].z=y;
    }
    // char *Path= new char[100];
    // sprintf(Path,"/home/wp/vr_ws/map_campus_release1/map_campus%d.pcd",cloudNum);
    // string nPath(Path);
    // pcl::io::savePCDFileASCII(nPath,*laserCloudRegistered2);
    cloudNum++;

    pcl::PointCloud<pcl::PointXYZ> Odometry(2, 1);
    Odometry.points[0].x=transformSum2[3];
    Odometry.points[0].y=transformSum2[4];
    Odometry.points[0].z=transformSum2[5];
    Odometry.points[1].x=roll;
    Odometry.points[1].y=-pitch;
    Odometry.points[1].z=-yaw;
    // char *Path1= new char[100];
    // sprintf(Path1,"/home/wp/vr_ws/odometry_campus_release1/odometry_campus%d.pcd",odometryNum);
    // string nPath1(Path1);
    // pcl::io::savePCDFileASCII(nPath1,Odometry);
    odometryNum++;

    sensor_msgs::PointCloud2 laserCloudRegistered3;
    pcl::toROSMsg(*laserCloudRegistered2, laserCloudRegistered3);
    laserCloudRegistered3.header.stamp = ros::Time().fromSec(timeCloudRegistered);
    laserCloudRegistered3.header.frame_id = "/camera_init";

    pubLaserCloudRegistered2Pointer->publish(laserCloudRegistered3);//wjx 轨迹
}

void odomIntegratedHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{

    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    transformSum[0] = -pitch;
    transformSum[1] = -yaw;
    transformSum[2] = roll;

    transformSum[3] = laserOdometry->pose.pose.position.x;
    transformSum[4] = laserOdometry->pose.pose.position.y;
    transformSum[5] = laserOdometry->pose.pose.position.z;

    transformSum2[0]=transformSum[2];
    transformSum2[1]=transformSum[0];
    transformSum2[2]=transformSum[1];
    transformSum2[3]=transformSum[5];
    transformSum2[4]=transformSum[3];
    transformSum2[5]=transformSum[4];

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw
              (transformSum2[2], -transformSum2[0], -transformSum2[1]);
geometry_msgs::PoseStamped this_pose_stamped;

    laserOdometry2.header.stamp = laserOdometry->header.stamp;
    laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
    laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
    laserOdometry2.pose.pose.orientation.z = geoQuat.x;
    laserOdometry2.pose.pose.orientation.w = geoQuat.w;
    //以上一系列变换，其实就是将orientation.x=roll;orientation.y=-pitch;orientation.z=-yaw;
   this_pose_stamped.pose.position.x=transformSum2[3];
     this_pose_stamped.pose.position.y=transformSum2[4];
       this_pose_stamped.pose.position.z=transformSum2[5];
    laserOdometry2.pose.pose.position.x=transformSum2[3];
    laserOdometry2.pose.pose.position.y=transformSum2[4];
    laserOdometry2.pose.pose.position.z=transformSum2[5];


    trajectory.header.frame_id="/camera_init";
/****************wjx ************************/
     this_pose_stamped.pose.orientation.x =  -geoQuat.y;
     this_pose_stamped.pose.orientation.y =  -geoQuat.z;
     this_pose_stamped.pose.orientation.z =  geoQuat.x;
     this_pose_stamped.pose.orientation.w =  geoQuat.w;
     this_pose_stamped.header.stamp=laserOdometry->header.stamp;
     this_pose_stamped.header.frame_id="/camera_init";
     trajectory.poses.push_back(this_pose_stamped);
     pubpath->publish(trajectory);
       //pubLaserOdometry2Pointer->publish(laserOdometry2);


    //wzy
    Eigen::Quaterniond rotat = Eigen::Quaterniond(geoQuat.w,-geoQuat.y,-geoQuat.z,geoQuat.x);
    rotat = lastRotat.inverse() * rotat;
    Eigen::Vector3d translate (transformSum2[3]-lastTrans[0],transformSum2[4]-lastTrans[1],transformSum2[5]-lastTrans[2]);
    translate = lastRotat.inverse() * translate;
    odo_msg.pose.pose.orientation.x = rotat.x();
    odo_msg.pose.pose.orientation.y = rotat.y();
    odo_msg.pose.pose.orientation.z = rotat.z();
    odo_msg.pose.pose.orientation.w = rotat.w();
    odo_msg.pose.pose.position.x = translate[0];
    odo_msg.pose.pose.position.y = translate[1];
    odo_msg.pose.pose.position.z = translate[2];
    odo_msg.header.stamp=laserOdometry->header.stamp;
    odo_msg.header.frame_id="/camera_init";
    pubLaserOdometry2Pointer->publish(odo_msg);

    //wzy
    lastRotat = Eigen::Quaterniond(geoQuat.w,-geoQuat.y,-geoQuat.z,geoQuat.x);
    lastTrans = Eigen::Vector3d(transformSum2[3],transformSum2[4],transformSum2[5]);

    laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
    laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    laserOdometryTrans2.setOrigin(tf::Vector3(transformSum2[3], transformSum2[4], transformSum2[5]));

     world_imu.stamp_ = laserOdometry->header.stamp;
     world_imu.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
     //world_imu.setRotation(tf::Quaternion(geoQuat.z, geoQuat.x, geoQuat.y, geoQuat.w));
     world_imu.setOrigin(tf::Vector3(transformSum2[3], transformSum2[4], transformSum2[5]));

     vel_imu.stamp_ = laserOdometry->header.stamp;
     vel_imu.setRotation(tf::Quaternion(0, 0, 0, 1));

     vel_imu.setOrigin(tf::Vector3(0, 0, 0));

    tfBroadcaster2Pointer->sendTransform(laserOdometryTrans2);
    /////////////////////////////////////////////////////
    tfBroadcaster2Pointer1->sendTransform(world_imu);
    tfBroadcaster2Pointer2->sendTransform(vel_imu);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transCoordinate");
  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",2);
  ros::Subscriber subLaserCloudSurround = nh.subscribe<sensor_msgs::PointCloud2>
                                     ("/laser_cloud_surround", 5, laserCloudSurroundHandler);

  ros::Subscriber subOdomIntegrated = nh.subscribe<nav_msgs::Odometry>
                                     ("/integrated_to_init", 5, odomIntegratedHandler);

  ros::Subscriber subCloudRegistered = nh.subscribe<sensor_msgs::PointCloud2>
                                     ("/velodyne_cloud_registered",2,CloudRegisteredHandler);

  ros::Publisher pubLaserCloudSurround2=nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_surround2",2);
  ros::Publisher pubLaserCloudRegistered2=nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_registered2",2);
  ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/integrated_to_init2", 5);
  ros::Publisher pubM = nh.advertise<nav_msgs::OccupancyGrid> ("/map", 1);

  pubLaserCloudSurround2Pointer = &pubLaserCloudSurround2;
  pubLaserCloudRegistered2Pointer = &pubLaserCloudRegistered2;
  pubLaserOdometry2Pointer = &pubLaserOdometry2;
  pubpath=&path_pub;
  pubMap = &pubM;

  laserOdometry2.header.frame_id = "/camera_init";
  laserOdometry2.child_frame_id = "/camera";

  tf::TransformBroadcaster tfBroadcaster2;
  tfBroadcaster2Pointer = &tfBroadcaster2;

  tf::TransformBroadcaster tfBroadcaster21;
  tfBroadcaster2Pointer1 = &tfBroadcaster21;
  tf::TransformBroadcaster tfBroadcaster22;
  tfBroadcaster2Pointer2 = &tfBroadcaster22;

  laserOdometryTrans2.frame_id_ = "/camera_init";
  laserOdometryTrans2.child_frame_id_ = "/camera";

    world_imu.frame_id_="world";world_imu.child_frame_id_="imu";
    vel_imu.frame_id_="imu"; vel_imu.child_frame_id_="velodyne";


  int map_size = 600;
  double map_resolution = 0.05;
  geometry_msgs::Pose map_origin;
  map_2d.header.frame_id = "/map";
  map_2d.info.height = map_size;
  map_2d.info.width = map_size;
  map_2d.info.resolution = map_resolution;
  map_2d.info.origin.position.x = double(map_2d.info.width)*map_resolution*-0.5;
  map_2d.info.origin.position.y = double(map_2d.info.height)*map_resolution*-0.5;
  map_2d.info.origin.position.z = 0.0;
  map_2d.info.origin.orientation.w = 1.0;
  map_2d.info.origin.orientation.x = 0.0;
  map_2d.info.origin.orientation.y = 0.0;
  map_2d.info.origin.orientation.z = 0.0;
  map_2d.data.resize(map_2d.info.height*map_2d.info.width);
  int index = 0;
  for(int i = 0; i < map_2d.info.width; i++)
  {
      for(int j = 0; j < map_2d.info.height; j++)
      {
          map_2d.data[index] = 0;
          index++;
      }
  }
  ros::spin();

  return 0;
}

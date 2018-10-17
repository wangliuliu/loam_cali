#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <iostream>
#include <string>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

//#include <QCoreApplication>
//#include <QDir>
//#include <QString>
//#include <QFile>
//#include <QDebug>
//#include <QTextStream>

using namespace std;

int user_data;
pcl::visualization::CloudViewer viewer("Cloud Viewer");


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(1.0,0.5,1.0);
    pcl::PointXYZ o;
    o.x=1.0;
    o.y=0;
    o.z=0;
    viewer.addSphere(o,0.25,"sphere",0);
    std::cout<<"i only run once"<<std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count=0;
    std::stringstream ss;
    ss<<"Once per viewer loop:"<<count++;
    viewer.removeShape("text",0);
    viewer.addText(ss.str(),200,300,"text",0);
    user_data++;
}

//qint64 GetFolderSize(QString folder)
//{
//    QDir dir(folder);
//    if (!dir.exists())
//        return 0;
//    qint64 totalsize = 0;
//    QFileInfoList list = dir.entryInfoList();
//    for (int i=0; i<list.size(); i++)
//    {
//        QFileInfo fileInfo = list.at(i);
//        if (fileInfo.isFile())
//            totalsize += fileInfo.size();
//        else if (fileInfo.isDir())
//        {
//            totalsize += GetFolderSize(fileInfo.path());
//        }
//    }
//    return totalsize;
//}

main( int argc , char **argv)
{
    ros::init(argc,argv,"map_visualization");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr laserCloudSurround2(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr laserCloudSurround3(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::VoxelGrid<pcl::PointXYZRGBA> downSizeFilterMap;
    downSizeFilterMap.setLeafSize(1.0f, 1.0f, 1.0f);

    pcl::io::loadPCDFile("/home/wp/map_write2.pcd",*cloud1);

//    DIR *DP;
//    struct dirent *dirp;
//    int n=0;
//    char *FPath = new char[100];
//    sprintf(FPath,"/home/wp/vr_ws/map_campus_release1");
//    string map_path(FPath);
//    if((DP=opendir(argv[1]))==NULL)
//        printf("cannot open: %s\n",argv[1]);
//    else
//        printf("open: %s\n",argv[1]);
//    while(((dirp=readdir(DP))))
//    {
//        n++;
//        if(n>2)
//        {
//            printf("file name: %s\n",dirp->d_name);
//            char *Path = new char[100];
//            char *Name = new char[100];
//            sprintf(Path,argv[1]);
//            sprintf(Name,dirp->d_name);
//            strcat(Path,Name);
//            string nPath(Path);
//            std::cout<< nPath<<std::endl;
//            pcl::io::loadPCDFile(nPath,*cloud1);
//            *laserCloudSurround2 += *cloud1;
//        }

//    }
//    printf("n=%d\n",n);
//    closedir(DP);
////    for (int i=0; i<n ;i++)
////    {
////        char *Path= new char[100];
////        sprintf(Path,"/home/wp/vr_ws/map_campus_ekf/map_campus%d.pcd",i);
////        string nPath(Path);
////        pcl::io::loadPCDFile(nPath,*cloud1);
////        *laserCloudSurround2 += *cloud1;
////    }
//    downSizeFilterMap.setInputCloud(laserCloudSurround2);
//    ROS_INFO("size2=%d\n",laserCloudSurround2->points.size());
//    downSizeFilterMap.filter(*laserCloudSurround3);
//    ROS_INFO("size3=%d\n",laserCloudSurround3->points.size());
//    pcl::io::savePCDFileASCII(argv[2],*laserCloudSurround3);
//    viewer.showCloud(laserCloudSurround3);
    viewer.showCloud(cloud1);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    viewer.runOnVisualizationThread(viewerPsycho);

//    pcl::io::loadPCDFile("/home/wp/vr_ws/map_campus0/map_campus0.pcd",*cloud1);
//    viewer.showCloud(cloud1);
//    viewer.runOnVisualizationThreadOnce(viewerOneOff);
//    viewer.runOnVisualizationThread(viewerPsycho);
    while(!viewer.wasStopped())
    {
        user_data++;
    }  //显示pcd文件
    ros::spin();
    return 0;
}

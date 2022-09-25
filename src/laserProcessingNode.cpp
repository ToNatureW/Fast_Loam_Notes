/*******************************
 作者：wwdream@126.com
 github:https://github.com/ToNatureW/Fast_Loam_Notes
 --------------------------------
 功能简介：
    1、初始化雷达的参数；
    2、订阅激光雷达原始的数据；
    3、特征点提取

 订阅话题：
    1、激光雷达原始数据

 发布话题：
    1、所有特征点（角点和平面点）
    2、角点
    3、平面点
********************************/


#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;  //　队列中保存原始雷达的每一帧点云
lidar::Lidar lidar_param;

ros::Publisher pubEdgePoints;           //　发布一帧的特征点
ros::Publisher pubSurfPoints;           //  发布一帧的面点
ros::Publisher pubLaserCloudFiltered;   // 发布一帧所有特征点的集合，没有降采样

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) //　将每一帧激光雷达数据加入到队列中
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
   
}

double total_time =0;   //　处理每一帧的特征点所需时间的总和
int total_frame=0;      //一共处理了多少帧

//
void laser_processing(){
    while(1){
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());  // 历史帧队列中最前面的一帧，时间最早
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp; // 取时间最早的一帧，最前面一帧的时间戳
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;  // 计算提取特征点的消耗时间
            start = std::chrono::system_clock::now();
            laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf); // 每一帧激光雷达点云的角点和面点特征提取,没有降采样
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;   // 提取一帧的特征点所需时间
            total_time += time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;     // 发布角点和面点的集合，没有降采样
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());   //　每一帧点云的角点和面点集合，后续发布
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;     // 发布一帧的角点
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;     // 发布一帧的面点
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);      //　休眠2ms
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;  //激光雷达线束
    double vertical_angle = 2.0;  //雷达的垂直分辨率
    double scan_period= 0.1;    //扫描周期
    double max_dis = 60.0;      //雷达扫描点的最大距离阈值
    double min_dis = 2.0;       //雷达扫描点的最小距离阈值

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);

    //初始化雷达的参数
    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    //将雷达的参数，传给processing的类中
    laserProcessing.init(lidar_param);

    //　接收原始雷达点云数据，
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);


    // 发布一帧所有特征点的集合，没有降采样
    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);
    //　发布一帧的特征点
    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);
    //  发布一帧的面点
    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100);

    // 提取特征点，并发布
    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}


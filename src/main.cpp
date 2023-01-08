#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"
#include <aidin_msgs/HeightMapPla.h> // son
#include <aidin_msgs/height.h> // son

using namespace cv;
using namespace std;


//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
// Publisher
ros::Publisher mask_pub;
ros::Publisher path_pub;
ros::Publisher occupancy_pub; // son
ros::Publisher height_pub; // son

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;

// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
int rate;
bool path_flag; // son

//-------------------------------- Callback function ---------------------------------//
void MapCallback(const aidin_msgs::HeightMapPla& msg)
{
    nav_msgs::OccupancyGrid occupancy_grid_map;
    occupancy_grid_map = msg.occupancy;
    occupancy_pub.publish(occupancy_grid_map); // 확인용
    
    aidin_msgs::height height_grid_map;
    height_grid_map = msg.height;
    height_pub.publish(height_grid_map); // 확인용

    // Get parameter
    OccGridParam.GetOccupancyGridParam(occupancy_grid_map);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = occupancy_grid_map.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);

    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "world";
    OccGridMask.info = occupancy_grid_map.info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }
    if(path_flag) {
    	// Set flag
    	map_flag = true;
    	startpoint_flag = false;
    	targetpoint_flag = false; 
	    path_flag = false;
    }
    //else { // 경로를 한번만 생성하려면 사용
    //    map_flag = false;
    // }
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint);


    // Set flag
    startpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
    //ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y, startPoint.x, startPoint.y);
}

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);

    // Set flag
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
    //ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y, targetPoint.x, targetPoint.y);
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;
    path_flag = true;

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 10);

    // Subscribe topics
    map_sub = nh.subscribe("height_map_pla", 10, MapCallback); // octomap에서 받는 2.5D height map
    startPoint_sub = nh.subscribe("initialpose", 10, StartPointCallback); // rviz에서 받는 start point
    targetPoint_sub = nh.subscribe("move_base_simple/goal", 10, TargetPointtCallback); // rviz에서 받는 target point

    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1); // rviz로 보내는 2D inflated map
    path_pub = nh.advertise<nav_msgs::Path>("nav_path", 10); // rviz로 보내는 path

    // octomap에서 받은 2.5D height map을 occupancy와 height로 나누어 publish. 확인 용도
    occupancy_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy", 10);
    height_pub = nh.advertise<aidin_msgs::height>("height", 10);

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {

        if(start_flag)
        {                          
            double start_time = ros::Time::now().toSec();
            // Start planning path
            vector<Point> PathList;	
            astar.PathPlanning(startPoint, targetPoint, PathList); // 0

            if(!PathList.empty())
            {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "world";
                path.poses.clear();
                for(int i=0;i<PathList.size();i++)
                {
                    Point2d dst_point;
                    OccGridParam.Image2MapTransform(PathList[i], dst_point);
                    
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "world";
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);
                double end_time = ros::Time::now().toSec();

                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
		        path_flag = true; // 경로를 한번만 생성하려면 주석
                
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            start_flag = false;
        }

        if(map_flag)
        {
            mask_pub.publish(OccGridMask);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}


/*
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <aidin_msgs/HeightMapPla.h>
#include <aidin_msgs/height.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"


using namespace cv;
using namespace std;


//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
// Publisher
ros::Publisher mask_pub;
ros::Publisher path_pub;
ros::Publisher occupancy_pub; //son
ros::Publisher height_pub; //son

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;

// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
int rate;

//-------------------------------- Callback function ---------------------------------//
void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    
    // Get parameter
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);

    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "world";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = Mask.at<uchar>(height-i-1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) // start point callback function
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint);

    // Set flag
    startpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

//    ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y,
//             startPoint.x, startPoint.y);
}

void TargetPointtCallback(const geometry_msgs::PoseStamped& msg) // target point callback function
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);

    // Set flag
    ROS_INFO("%d", startpoint_flag);
    ROS_INFO("%d", map_flag);
    
    targetpoint_flag = true;
    if(map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

//    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
//             targetPoint.x, targetPoint.y);
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 10);

    // Subscribe topics
    map_sub = nh.subscribe("projected_map", 10, MapCallback); //son
    startPoint_sub = nh.subscribe("initialpose", 10, StartPointCallback);
    targetPoint_sub = nh.subscribe("move_base_simple/goal", 10, TargetPointtCallback);

    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1);
    path_pub = nh.advertise<nav_msgs::Path>("nav_path", 10);
    //occupancy_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy", 10);
    //height_pub = nh.advertise<aidin_msgs::height>("height", 10);

    // Loop and wait for callback
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        if(start_flag)
        {
            double start_time = ros::Time::now().toSec();
            // Start planning path
            vector<Point> PathList;
            astar.PathPlanning(startPoint, targetPoint, PathList);
            if(!PathList.empty())
            {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "world";
                path.poses.clear();
                for(int i=0;i<PathList.size();i++)
                {
                    Point2d dst_point;
                    OccGridParam.Image2MapTransform(PathList[i], dst_point);

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "world";
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);
                double end_time = ros::Time::now().toSec();

                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            start_flag = false;
        }

        if(map_flag)
        {
            mask_pub.publish(OccGridMask);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}*/


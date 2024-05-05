#ifndef VISUALIZATION_NODE_H
#define VISUALIZATION_NODE_H


#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"


struct Robot {
   float posX;
   float posY;
   float orientation;
   float sizeX;
   float sizeY;
};


class VisualizationNode {
public:
   VisualizationNode();
   ~VisualizationNode();
   void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
   void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
   void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);


   void visualizeMap();
   void updateRobotPosition(float x, float y, float orientation);
   void visualizeRobot();
   void visualizeScan();
   void onMouse(int event, int x, int y, int flags);

   static void onMouseStatic(int event, int x, int y, int flags, void* userdata);


private:

  bool initial_pose_set;
   ros::NodeHandle nh;
   ros::Subscriber map_sub;
   ros::Subscriber odom_sub;
   ros::Subscriber sub;

   ros::Publisher initial_pose_pub;
   ros::Publisher goal_pub;


   nav_msgs::OccupancyGrid::ConstPtr map_msg;
   sensor_msgs::LaserScan::ConstPtr msg;
   cv::Mat map_image;
   bool map_initialized;

   std::vector<Robot> robots;


   cv::Point stageToOpenCV(float x_stage, float y_stage);
   cv::Point clickPointToStage(const cv::Point& click_point);


   int height; //Height of the map
   void publishInitialPose(const cv::Point& point);
   void publishGoal(const cv::Point& point);



};


#endif // VISUALIZATION_NODE_H











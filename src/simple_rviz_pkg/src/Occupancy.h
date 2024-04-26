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



struct Ostacolo {
    std::string nome;
    std::string colore;
    float posX;
    float posY;
    float posZ;
    float orientation;
    float sizeX;
    float sizeY;
};

struct Robot {
    std::string nome;
    std::string colore;
    float posX;
    float posY;
    float posZ;
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
    void readWorldFile(const std::string& filename);
    void updateRobotPosition(float x, float y, float orientation);
    void getRobotPosition(float& x, float& y, float& orientation);
    void visualizeRobot();

private:

    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber sub;

    nav_msgs::OccupancyGrid::ConstPtr map_msg;
    cv::Mat map_image;
    bool map_initialized;

    std::vector<Ostacolo> ostacoli;
    std::vector<Robot> robots;

    cv::Point stageToOpenCV(float x_stage, float y_stage);
    int height; //Height of the map

};

#endif // VISUALIZATION_NODE_H




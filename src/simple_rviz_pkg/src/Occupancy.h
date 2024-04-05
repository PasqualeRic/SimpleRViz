#ifndef VISUALIZATION_NODE_H
#define VISUALIZATION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>

struct Robot {
    float posX;
    float posY;
    float posZ;
    float sizeX;
    float sizeY;
    float sizeZ;
    float orientation;
    std::string colore;
    std::string nome;
};

struct Ostacolo {
    float posX;
    float posY;
    float posZ;
    float sizeX;
    float sizeY;
    float sizeZ;
    float orientation;
    std::string colore;
    std::string nome;
};

class VisualizationNode {
public:
    VisualizationNode();
    ~VisualizationNode();

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    cv::Point stageToOpenCV(float x_stage, float y_stage);
    void visualizeMap();
    void readWorldFile(const std::string& filename);

private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    nav_msgs::OccupancyGrid::ConstPtr map_msg;
    std::vector<Ostacolo> ostacoli;
    std::vector<Robot> robots;
    bool map_initialized;
};

#endif // VISUALIZATION_NODE_H




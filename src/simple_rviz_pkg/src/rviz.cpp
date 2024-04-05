#include "Occupancy.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualization_node");
    ROS_INFO("Map listener node started");

    VisualizationNode node;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        node.visualizeMap();
        rate.sleep();
    }

    return 0;
}




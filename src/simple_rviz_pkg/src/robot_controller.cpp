#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_and_goal_publisher");
    ros::NodeHandle nh;

    ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::Publisher goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);

    ros::Duration(2.0).sleep();

    double initial_pose_x, initial_pose_y;
    ROS_INFO("Inserisci le coordinate della posizione iniziale (x y): ");
    std::cin >> initial_pose_x >> initial_pose_y;



    // Create and publishes the initial position
    geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
    initial_pose_msg.header.stamp = ros::Time::now();
    initial_pose_msg.header.frame_id = "map"; 
    initial_pose_msg.pose.pose.position.x = -(initial_pose_x)+(106.913/2); 
    initial_pose_msg.pose.pose.position.y = -(initial_pose_y)+(49.3527/2);  
    initial_pose_msg.pose.pose.position.z = 0.0; 
    initial_pose_msg.pose.pose.orientation.w = 180.000; 

    initial_pose_pub.publish(initial_pose_msg);
    ROS_INFO("New initial pose published.");
    
    double goal_x, goal_y;
    ROS_INFO("Inserisci le coordinate del goal (x y): ");
    std::cin >> goal_x >> goal_y;



    // Create and publishes the goal
    move_base_msgs::MoveBaseActionGoal goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";  
    goal_msg.goal.target_pose.header.stamp = ros::Time::now();
    goal_msg.goal.target_pose.header.frame_id = "map"; 
    goal_msg.goal.target_pose.pose.position.x = -(goal_x)+(106.913/2); 
    goal_msg.goal.target_pose.pose.position.y = -(goal_y)+(49.3527/2); 
    goal_msg.goal.target_pose.pose.position.z = 0.0; 
    goal_msg.goal.target_pose.pose.orientation.w = 180.000;

    goal_pub.publish(goal_msg);
    ROS_INFO("New goal published.");

    ros::spin();

    return 0;
}




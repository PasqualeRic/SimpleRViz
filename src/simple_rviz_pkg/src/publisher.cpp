#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher_node");
    ros::NodeHandle nh;
	ROS_INFO("prova movebase");
    // Publisher per il topic del goal
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
ROS_INFO("prova pose with covariance");
    // Publisher per il topic dell'initial guess
    ros::Publisher initial_guess_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialguess", 1);

    // Creazione dei messaggi del goal e dell'initial guess
    geometry_msgs::PoseStamped goal_msg;
    geometry_msgs::PoseWithCovarianceStamped initial_guess_msg;
ROS_INFO("prova imposta position goal");
    // Impostazione della posizione desiderata per il goal
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = 10.0;
    goal_msg.pose.position.y = 5.0;
    goal_msg.pose.orientation.w = 1.0;
ROS_INFO("prova initial guess");
    // Impostazione della posizione iniziale stimata (initial guess)
    initial_guess_msg.header.stamp = ros::Time::now();
    initial_guess_msg.header.frame_id = "map";
    initial_guess_msg.pose.pose.position.x = 2.0;
    initial_guess_msg.pose.pose.position.y = 2.0;
    initial_guess_msg.pose.pose.orientation.w = 1.0;
ROS_INFO("prova pub msg");
    // Pubblicazione dei messaggi sul rispettivo topic
    goal_pub.publish(goal_msg);
    initial_guess_pub.publish(initial_guess_msg);

    ros::Duration(1).sleep();
    ros::spin();

    return 0;
}




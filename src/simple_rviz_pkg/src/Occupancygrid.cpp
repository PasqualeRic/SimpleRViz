#include "Occupancy.h"

VisualizationNode::VisualizationNode() : nh("~"){
    // Subscription to the topics
    map_sub = nh.subscribe("/map", 1000, &VisualizationNode::mapCallback, this);
    odom_sub = nh.subscribe("/odom", 1000, &VisualizationNode::odomCallback, this); // Aggiunta della sottoscrizione all'odometria
    sub = nh.subscribe("/base_scan", 1000, &VisualizationNode::scanCallback, this);

    // Initialization of the visualization window
    cv::namedWindow("Occupancy Grid Map", cv::WINDOW_NORMAL);
    cv::resizeWindow("Occupancy Grid Map", 1900, 800);
    //Publisher initial pose and goal
    initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    cv::setMouseCallback("Occupancy Grid Map", onMouseStatic, (void*)this);


}

VisualizationNode::~VisualizationNode() {
    cv::destroyWindow("Occupancy Grid Map");
}
void VisualizationNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_msg = msg;
    ROS_INFO("Received occupancy grid map");
    visualizeMap();
}
//This function is used to convert the stage coordinates into the opencv
cv::Point VisualizationNode::stageToOpenCV(float x_stage, float y_stage) {
    // Dimension of the Opencv window
    float width_opencv = x_stage * 20 + 2138 / 2;
    float height_opencv = y_stage * 20 + 987 / 2;
    return cv::Point(width_opencv, height_opencv);
}
//This functon show the map with the obstacles, robot and laser
void VisualizationNode::visualizeMap() {
    //if the map server is not online or the world file has not been read, skip
    if (!map_msg) {
        ROS_WARN("Map not initialized. Skipping visualization.");
        return;
    }

    if (map_msg) {
        int width = map_msg->info.width;
        height = map_msg->info.height;

        // Draw the image 
        map_image = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255)); // White
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                int value = map_msg->data[index];

                cv::Vec3b color;
                if (value == -1)
                    color = cv::Vec3b(0, 0, 0); //Black for unknown
                else if (value == 0)
                    color = cv::Vec3b(255, 255, 255); // White for free
                else
                    color = cv::Vec3b(0, 0, 255); // Red for occupate

                // This allows to reverse the map on the y axes
                map_image.at<cv::Vec3b>(cv::Point(x, height - 1 - y)) = color;
            }
        }
       visualizeRobot();
       visualizeScan();
        //Visualize the image 
        cv::imshow("Occupancy Grid Map", map_image);
        cv::waitKey(1);
    }
}


void VisualizationNode::visualizeRobot() {
    for (const auto& robot : robots) {
        //Coordinates of the robot
        float x_stage = robot.posX;
        float y_stage = robot.posY;

        cv::Point position = stageToOpenCV(x_stage, y_stage);
        //Reverse coordinates
	    position.y= height - 1 - position.y;
        int sizeX = static_cast<int>(0.4 * 20); 
        int sizeY = static_cast<int>(0.4 * 20);

            cv::Scalar color;
                color = cv::Scalar(255, 0, 0); // Blue

            cv::circle(map_image, position, sizeX / 2, color, -1); // Draw the circle 
    }
}



//This function update the robot position
void VisualizationNode::updateRobotPosition(float x, float y, float orientation) {
	if(!robots.empty()){
		robots[0].posX=x;
		robots[0].posY=y;
		robots[0].orientation=orientation;
	}
   
}
//This function get the position of the robot, set the struct robot and update the position 
void VisualizationNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    float theta = tf::getYaw(msg->pose.pose.orientation);
if(robots.empty()){
    Robot robot;
    robot.posX = x;
    robot.posY = y;
    robot.orientation = theta;

    // Pusha il nuovo robot nel vettore di robots
    robots.push_back(robot);
}


    // Aggiorna la posizione del robot
   updateRobotPosition(x, y, theta);
}
 
void VisualizationNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg_s) {
   msg = msg_s;
}
void VisualizationNode::visualizeScan(){
	if (!map_msg || robots.empty()) {
        ROS_WARN("Map or robot not initialized. Skipping scan visualization.");
        return;
    }

    float robot_x = robots[0].posX;
    float robot_y = robots[0].posY;
    float robot_orientation = robots[0].orientation;

    std::vector<cv::Point> scan_points;
    //This for get all the values in msg and calculate the position respect the orientation of the robot
    // also in this case the coordinates are plotted 
    for (int i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        float angle = msg->angle_min + i * msg->angle_increment;
        //adjuste the angle to the robot 
        angle += M_PI;
        if (range < msg->range_min || range > msg->range_max) {
            continue;
        }
        //calculate the correct angle
        float adjusted_angle = angle + robot_orientation - M_PI; 
        //this calculate the coordinates 
        float x_stage = robot_x + range * cos(adjusted_angle);
        float y_stage = robot_y + range * sin(adjusted_angle);
        
        cv::Point point_opencv = stageToOpenCV(x_stage, y_stage);
        //plot on the y axes
        point_opencv.y = map_msg->info.height - 1 - point_opencv.y;
        //draw the point
        cv::circle(map_image, point_opencv, 2, cv::Scalar(0, 255, 0), cv::FILLED);
    }
    cv::imshow("Occupancy Grid Map", map_image);
    cv::waitKey(1);
}
cv::Point VisualizationNode::clickPointToStage(const cv::Point& click_point) {
    // Calcola le coordinate x e y del punto cliccato rispetto alla finestra OpenCV
    int x_opencv = click_point.x;
    int y_opencv = click_point.y;

    // Calcola le coordinate x e y del punto cliccato rispetto alla mappa di stage
    float x_stage = (x_opencv - 2138 / 2) / 20.0;
    float y_stage = (y_opencv - 987 / 2) / 20.0;

    return cv::Point(x_stage, -y_stage);
}

void VisualizationNode::onMouse(int event, int x, int y, int flags) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Point click_point(x, y);
        cv::Point stage_point = clickPointToStage(click_point);
        publishInitialPose(stage_point);
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        cv::Point click_point(x, y);
        cv::Point stage_point = clickPointToStage(click_point);
        publishGoal(stage_point);
    }
}

//Questo wrapper statico consente di passare un puntatore all'istanza della classe VisualizationNode
void VisualizationNode::onMouseStatic(int event, int x, int y, int flags, void* userdata) {
    VisualizationNode* instance = static_cast<VisualizationNode*>(userdata);
    instance->onMouse(event, x, y, flags);
}

void VisualizationNode::publishInitialPose(const cv::Point& point) {
	
    if (!initial_pose_set) {
        geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = ros::Time::now();
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.pose.pose.position.x = -(point.x) + (106.913 / 2);
        initial_pose_msg.pose.pose.position.y = -(point.y) + (49.3527 / 2);
        initial_pose_msg.pose.pose.position.z = 0.0;
        initial_pose_msg.pose.pose.orientation.w = 180.000;
        std::cout<<"x"<<point.x<<"y:"<<point.y<<std::endl;
        initial_pose_pub.publish(initial_pose_msg);
        ROS_INFO("New initial pose published.");
        initial_pose_set = true;
    }
}

void VisualizationNode::publishGoal(const cv::Point& point) {
    move_base_msgs::MoveBaseActionGoal goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";
    goal_msg.goal.target_pose.header.stamp = ros::Time::now();
    goal_msg.goal.target_pose.header.frame_id = "map";
    goal_msg.goal.target_pose.pose.position.x = -(point.x) + (106.913 / 2);
    goal_msg.goal.target_pose.pose.position.y = -(point.y) + (49.3527 / 2);
    goal_msg.goal.target_pose.pose.position.z = 0.0;
    goal_msg.goal.target_pose.pose.orientation.w = 180.000;
    std::cout<<"x"<<point.x<<"y:"<<point.y<<std::endl;
    goal_pub.publish(goal_msg);
    ROS_INFO("New goal published.");
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "visualization_node");
    ROS_INFO("Map listener node started");

    VisualizationNode node;

    ros::NodeHandle nh;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        // Visualizza la mappa
        node.visualizeMap();

        rate.sleep();
    }
    return 0;
}










































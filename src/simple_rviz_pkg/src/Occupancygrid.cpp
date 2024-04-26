#include "Occupancy.h"

VisualizationNode::VisualizationNode() : nh("~"){
    // Subscription to the topics
    map_sub = nh.subscribe("/map", 1000, &VisualizationNode::mapCallback, this);
    odom_sub = nh.subscribe("/odom", 1000, &VisualizationNode::odomCallback, this); // Aggiunta della sottoscrizione all'odometria
    sub = nh.subscribe("/base_scan", 1000, &VisualizationNode::scanCallback, this);

    // Initialization of the visualization window
    cv::namedWindow("Occupancy Grid Map", cv::WINDOW_NORMAL);
    cv::resizeWindow("Occupancy Grid Map", 1900, 800);
    readWorldFile("./map/cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world");
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
//This functon shows users the map with the obstacles and robot
void VisualizationNode::visualizeMap() {
    //if the map server is not online or the world file has not been read, skip
    if (!map_initialized || !map_msg) {
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

        // Draw the obstacles
        for (const auto& ostacolo : ostacoli) {
            cv::Point position = stageToOpenCV(ostacolo.posX, ostacolo.posY);

            // Apply the transformation to reverse the obstacles
            position.y = height - 1 - position.y;
            //Dimension scale
            int sizeX = static_cast<int>(ostacolo.sizeX * 20);
            int sizeY = static_cast<int>(ostacolo.sizeY * 20); 

            cv::Scalar color;
            if (ostacolo.colore == "red") {
                color = cv::Scalar(0, 0, 255); // Red
            } else if (ostacolo.colore == "purple") {
                color = cv::Scalar(255, 0, 255); //Purple
            }
            //Draw the obstacle on the image
            cv::rectangle(map_image, position - cv::Point(sizeX / 2, sizeY / 2),
                          position + cv::Point(sizeX / 2, sizeY / 2), color, -1); 
        }
        visualizeRobot();
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
        //Reverse like the obstacles
	    position.y= height - 1 - position.y;
        int sizeX = static_cast<int>(robot.sizeX * 20); 
        int sizeY = static_cast<int>(robot.sizeY * 20);

            cv::Scalar color;
            if (robot.colore == "blue") {
                color = cv::Scalar(255, 0, 0); // Blue
            }

            cv::circle(map_image, position, sizeX / 2, color, -1); // Draw the circle 
    }
}



//This function allow to read the world file and get the obstacles and robot coordinates before the map initialization
//Create the structs obstacle and robot
void VisualizationNode::readWorldFile(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;
        if (token == "block") {
            Ostacolo ostacolo;
            size_t pos;
            pos = line.find("name");
            ostacolo.nome = line.substr(pos + 6, line.find("\"", pos + 6) - (pos + 6));

            pos = line.find("color");
            ostacolo.colore = line.substr(pos + 7, line.find("\"", pos + 7) - (pos + 7));

            pos = line.find("pose");
            std::string pose_str = line.substr(pos + 6, line.find("]", pos + 6) - (pos + 6));
            std::istringstream pose_iss(pose_str);
            pose_iss >> ostacolo.posX >> ostacolo.posY >> ostacolo.posZ >> ostacolo.orientation;
            ostacolo.sizeX = 0.2;
            ostacolo.sizeY = 0.2;

            ostacoli.push_back(ostacolo);
                  
        } else if (token == "erratic") {
            Robot robot;
            size_t pos;
            pos = line.find("name");
            robot.nome = line.substr(pos + 6, line.find("\"", pos + 6) - (pos + 6));

            pos = line.find("color");
            robot.colore = line.substr(pos + 7, line.find("\"", pos + 7) - (pos + 7));

            pos = line.find("pose");
            std::string pose_str = line.substr(pos + 6, line.find("]", pos + 6) - (pos + 6));
            std::istringstream pose_iss(pose_str);
            pose_iss >> robot.posX >> robot.posY >> robot.posZ >> robot.orientation;
            robot.sizeX = 0.2;
            robot.sizeY = 0.2;

            robots.push_back(robot);
         
        }
    }
    file.close();

    map_initialized = true;
}
//This function update the robot position and recall the visualize robot function and print the new position on the image
void VisualizationNode::updateRobotPosition(float x, float y, float orientation) {
std::cout<<"x"<<x<<"y"<<y<<std::endl;
	if(!robots.empty()){
		robots[0].posX=x;
		robots[0].posY=y;
		robots[0].orientation=orientation;
	}
     
      visualizeRobot();
}
//This function get the position of the robot from stage and update position
void VisualizationNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    float theta = tf::getYaw(msg->pose.pose.orientation);
    // Aggiorna la posizione del robot
   updateRobotPosition(x, y, theta);


}
//This function printo on the image the point scan 
void VisualizationNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
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























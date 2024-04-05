#include "Occupancy.h"

VisualizationNode::VisualizationNode() : nh("~"), map_initialized(false) {
    // Sottoscrizione ai topic
    map_sub = nh.subscribe("/map", 1000, &VisualizationNode::mapCallback, this);
    // Inizializzazione della finestra di visualizzazione
    cv::namedWindow("Occupancy Grid Map", cv::WINDOW_NORMAL);
    cv::resizeWindow("Occupancy Grid Map", 2138, 987);
    readWorldFile("./map/cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world");
}

VisualizationNode::~VisualizationNode() {
    cv::destroyWindow("Occupancy Grid Map");
}

void VisualizationNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_msg = msg;
    ROS_INFO("Received occupancy grid map");
}

cv::Point VisualizationNode::stageToOpenCV(float x_stage, float y_stage) {
    // Dimensioni della finestra di OpenCV
    float width_opencv = x_stage * 20 + 2148 / 2;
    float height_opencv = y_stage * 20 + 987 / 2;
    // Calcola le coordinate in OpenCV
    return cv::Point(width_opencv, height_opencv);
}

void VisualizationNode::visualizeMap() {
    if (!map_initialized || !map_msg) {
        ROS_WARN("Map not initialized. Skipping visualization.");
        return;
    }

    if (map_msg) {
        // Ottieni le dimensioni della mappa
        int width = map_msg->info.width;
        int height = map_msg->info.height;

        // Disegna la mappa sull'immagine
        cv::Mat map_image(height, width, CV_8UC3, cv::Scalar(255, 255, 255)); // Bianco
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                int value = map_msg->data[index];

                cv::Vec3b color;
                if (value == -1)
                    color = cv::Vec3b(0, 0, 0); // Nero per sconosciuto
                else if (value == 0)
                    color = cv::Vec3b(255, 255, 255); // Bianco per libero
                else
                    color = cv::Vec3b(0, 0, 255); // Rosso per occupato

                map_image.at<cv::Vec3b>(cv::Point(x, y)) = color;
            }
        }

        //cv::rotate(map_image, map_image, cv::ROTATE_180);
        // Disegna gli ostacoli sull'immagine
        for (const auto& ostacolo : ostacoli) {
            cv::Point position = stageToOpenCV(ostacolo.posX, ostacolo.posY);
            ROS_INFO("Ostacolo: Posizione (Stage): (%f, %f), Posizione (OpenCV): (%d, %d)", ostacolo.posX, ostacolo.posY, position.x, position.y);

            int sizeX = static_cast<int>(ostacolo.sizeX * 20); // Scala la dimensione X dell'ostacolo
            int sizeY = static_cast<int>(ostacolo.sizeY * 20); // Scala la dimensione Y dell'ostacolo

            cv::Scalar color;
            if (ostacolo.colore == "red") {
                color = cv::Scalar(0, 0, 255); // Rosso per gli ostacoli
            } else if (ostacolo.colore == "purple") {
                color = cv::Scalar(255, 0, 255); // Viola per gli ostacoli
            }

            cv::rectangle(map_image, position - cv::Point(sizeX / 2, sizeY / 2),
                          position + cv::Point(sizeX / 2, sizeY / 2), color, -1); // Disegna il rettangolo degli ostacoli
        }

        // Disegna i robot sull'immagine
        for (const auto& robot : robots) {
            cv::Point position = stageToOpenCV(robot.posX, robot.posY);
            int sizeX = static_cast<int>(robot.sizeX * 20); // Scala la dimensione X del robot
            int sizeY = static_cast<int>(robot.sizeY * 20); // Scala la dimensione Y del robot

            cv::Scalar color;
            if (robot.colore == "blue") {
                color = cv::Scalar(255, 0, 0); // Blu per i robot
            }

            cv::circle(map_image, position, sizeX / 2, color, -1); // Disegna il cerchio rappresentante il robot
        }

        // Visualizza l'immagine della mappa utilizzando OpenCV
        cv::imshow("Occupancy Grid Map", map_image);
        cv::waitKey(1); // Attendiamo 1 millisecondo
    }
}

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
            ostacolo.sizeX = 0.2;  // Considera la dimensione dell'ostacolo come 0.2 metri
            ostacolo.sizeY = 0.2;

            ostacoli.push_back(ostacolo);
            ROS_INFO("Ostacolo letto: Nome: %s, Colore: %s, Posizione: (%2f, %2f, %2f), Orientamento: %f",
                     ostacolo.nome.c_str(), ostacolo.colore.c_str(),
                     ostacolo.posX, ostacolo.posY, ostacolo.posZ, ostacolo.orientation);
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
            robot.sizeX = 0.2;  // Considera la dimensione del robot come 0.2 metri
            robot.sizeY = 0.2;

            robots.push_back(robot);
            ROS_INFO("ROBOT letto: Nome: %s, Colore: %s, Posizione: (%2f, %2f, %2f), Orientamento: %f",
                     robot.nome.c_str(), robot.colore.c_str(),
                     robot.posX, robot.posY, robot.posZ, robot.orientation);
        }
    }
    file.close();

    map_initialized = true;
}




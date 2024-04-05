cmake_minimum_required(VERSION 3.0.2)
project(simple_rviz_pkg)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs)
find_package(OpenCV REQUIRED)
catkin_package(
	CATKIN_DEPENDS
	roscpp
	std_msgs)
add_executable(rviz src/rviz.cpp src/Occupancygrid.cpp)
target_include_directories(rviz PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(rviz ${catkin_LIBRARIES})
target_link_libraries(rviz ${OpenCV_LIBRARIES})

add_executable(publisher src/publisher.cpp)
target_include_directories(publisher PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(publisher ${catkin_LIBRARIES})
target_link_libraries(publisher ${OpenCV_LIBRARIES})
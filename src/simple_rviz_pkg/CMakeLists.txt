cmake_minimum_required(VERSION 3.0.2)
project(simple_rviz_pkg)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	move_base_msgs
	tf
	geometry_msgs
	actionlib
	sensor_msgs)
find_package(OpenCV REQUIRED)
catkin_package(
	CATKIN_DEPENDS
	roscpp
	geometry_msgs
	move_base_msgs
	actionlib
	std_msgs)
add_executable(rviz src/Occupancygrid.cpp )
target_include_directories(rviz PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(rviz ${catkin_LIBRARIES})
target_link_libraries(rviz ${OpenCV_LIBRARIES})
target_link_libraries(rviz ${Boost_LIBRARIES})
target_link_libraries(rviz ${tf_LIBRARIES})


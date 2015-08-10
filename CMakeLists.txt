set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(sonar_sensor)

find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	roscpp
	std_msgs
	message_generation
	pcl_conversions
  	pcl_ros
  	cv_bridge
)

#add_message_files(FILES
#	distPose.msg
#)

#generate_messages(DEPENDENCIES
#	std_msgs
#)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime
	INCLUDE_DIRS include
)
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(sonar_sensor src/sonar_sensor.cpp)
target_link_libraries(sonar_sensor ${catkin_LIBRARIES})
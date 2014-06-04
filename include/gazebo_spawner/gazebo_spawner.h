/*
 * gazebo_spawner.h
 *
 *  Created on: May 22, 2014
 *      Author: bcalli
 */

#ifndef GAZEBO_SPAWNER_H_
#define GAZEBO_SPAWNER_H_

#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "tinyxml.h"
//#include "gazebo_msgs/DeleteModel.h"
//#include "gazebo_msgs/SetModelState.h"

class GazeboSpawner{

	ros::NodeHandle* n_;
	ros::ServiceClient client_spawn_urdf_;
	bool initialized_;
	std::string model_path_;

public:
	GazeboSpawner();
	GazeboSpawner(ros::NodeHandle* n);
	void initialize(ros::NodeHandle* n);
	bool spawnObjectFromURDF(const char * name, float posx, float posy, float posz);
	bool spawnObjectFromURDF(const char * name, std::string file_name, float pos_x, float pos_y, float pos_z);
	bool spawnObjectFromURDF(const char * name, std::string file_name, float pos_x, float pos_y, float pos_z, float qx, float qy, float qz, float qw);
	bool spawnObjectFromURDF(const char * name, float pos_x, float pos_y, float pos_z, float qx, float qy, float qz, float qw);
	bool spawnObjects(std::vector<std::string> object_names);
	std::vector<double> getInitialPose(std::string object_name);
	void setPath(std::string path);
};


#endif /* GAZEBO_SPAWNER_H_ */

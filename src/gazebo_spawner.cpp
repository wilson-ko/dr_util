/*
 * gazebo_spawner.cpp
 *
 *  Created on: May 22, 2014
 *      Author: bcalli
 */

#include "gazebo_spawner/gazebo_spawner.h"

GazeboSpawner::GazeboSpawner(){
	n_ = NULL;
	initialized_ = false;
}

GazeboSpawner::GazeboSpawner(ros::NodeHandle* n){
	initialize(n_);
}

void GazeboSpawner::initialize(ros::NodeHandle* n){

	if(initialized_)
		return;

	n_ = n;
	client_spawn_urdf_ = n_->serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");

	if (!n_->getParam("model_path", model_path_)) {
		ROS_WARN("[bazo_sim_manager]: Failed to get the model_path parameter from the parameter server. Please set it manually.");
		model_path_ = "";
	}

	initialized_  = true;
	/*
	clientSpawnXML = n->serviceClient<gazebo::SpawnModel> (
			"gazebo/spawn_gazebo_model");
	client_delete_model_ = n->serviceClient<gazebo::DeleteModel> (
			"gazebo/delete_model");
	client_set_model_state_ = n->serviceClient<gazebo::SetModelState> (
			"gazebo/set_model_state");
	*/
}

void GazeboSpawner::setPath(std::string path){
	model_path_ = path;
}

bool GazeboSpawner::spawnObjectFromURDF(const char * name, float pos_x, float pos_y, float pos_z) {

	return spawnObjectFromURDF(name, name, pos_x,pos_y, pos_z);

}

bool GazeboSpawner::spawnObjects(std::vector<std::string> object_names) {

	std::vector<double> default_props (7,0.0); default_props[6] = 1;
	std::vector<double> object_pose;

	for (size_t i = 0; i<object_names.size(); i++){
		object_pose = getInitialPose(object_names[i]);

		spawnObjectFromURDF(object_names[i].c_str(), object_pose[0], object_pose[1], object_pose[2], object_pose[3], object_pose[4], object_pose[5], object_pose[6]);
	}

	return true;
}

std::vector<double> GazeboSpawner::getInitialPose(std::string object_name) {
	std::vector<double> default_pose (7,0.0); default_pose[6] = 1;

	XmlRpc::XmlRpcValue object_prop_xml;
	std::vector<double> object_pose;

	if (!n_->getParam("spawn_properties/"+object_name, object_prop_xml)) {
		ROS_WARN("[bazo_sim_manager]: Failed to get the model_path parameter from the parameter server. Using the default value: [%f %f %f %f %f %f %f]", default_pose[0], default_pose[1], default_pose[2], default_pose[3], default_pose[4], default_pose[5], default_pose[6]);
		object_pose = default_pose;
	}
	else {
		ROS_ASSERT(object_prop_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
		for (int32_t j = 0; j < object_prop_xml.size(); ++j) {
			ROS_ASSERT(object_prop_xml[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			object_pose.push_back(static_cast<double>(object_prop_xml[j]));
		}
	}
	return object_pose;
}

bool GazeboSpawner::spawnObjectFromURDF(const char * name, std::string file_name, float pos_x, float pos_y, float pos_z) {

	return spawnObjectFromURDF(name, file_name, pos_x, pos_y, pos_z, 0, 0, 0, 1);
}

bool GazeboSpawner::spawnObjectFromURDF(const char * name, float pos_x, float pos_y, float pos_z, float qx, float qy, float qz, float qw) {
	return spawnObjectFromURDF(name, name, pos_x, pos_y, pos_z, qx, qy, qz, qw);
}


bool GazeboSpawner::spawnObjectFromURDF(const char * name, std::string file_name, float pos_x, float pos_y, float pos_z, float qx, float qy, float qz, float qw) {

	if(!initialized_){
		ROS_WARN("GazeboSpawner is used before being initialized.");
		return false;
	}

	gazebo_msgs::SpawnModel srv_spawn;

	srv_spawn.request.model_name = name;
	//std::string file_name = model_path_ + name + ".urdf";
	//ROS_ERROR("file_name is : %s\n", file_name.c_str());
	TiXmlDocument xml_in(model_path_ + file_name + ".urdf");
	xml_in.LoadFile();
	std::ostringstream stream;
	stream << xml_in;
	srv_spawn.request.model_xml = stream.str();

	srv_spawn.request.robot_namespace = name;
	srv_spawn.request.initial_pose.position.x = pos_x;
	srv_spawn.request.initial_pose.position.y = pos_y;
	srv_spawn.request.initial_pose.position.z = pos_z;
	srv_spawn.request.initial_pose.orientation.x = qx;
	srv_spawn.request.initial_pose.orientation.y = qy;
	srv_spawn.request.initial_pose.orientation.z = qz;
	srv_spawn.request.initial_pose.orientation.w = qw;

	if (client_spawn_urdf_.call(srv_spawn)) {
		ROS_INFO("[ObjectSpawner]: \"%s\" has been spawned", name);
		return 1;
	} else {
		ROS_ERROR("[ObjectSpawner]: Failed to spawn \"%s\"", name);
		return 0;
	}

}


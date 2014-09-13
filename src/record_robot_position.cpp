#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <fstream>


int main(int argc, char** argv){
	ros::init(argc, argv, "record_robot_pos");
	ros::NodeHandle node;

	std::ofstream outfile;
	outfile.open("robot_pos.yaml");

	std::string parameter_name = "robot_pos";
	tf::TransformListener listener;
	tf::StampedTransform transform;
	listener.waitForTransform("ur_base_link", "ur_ee_link",      ros::Time(0),ros::Duration(10));
	try{
		listener.lookupTransform("ur_base_link", "ur_ee_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("The transform between /ur_base_link and /ur_ee_link cannot be obtained: %s",ex.what());
		return false;
	}

	// open a file in write mode.
	std::string data = parameter_name + ":\n  position: {x: " + boost::lexical_cast<std::string>(transform.getOrigin().x())+ ", ";
	data += "y: " + boost::lexical_cast<std::string>(transform.getOrigin().y()) + ", ";
	data += "z: " + boost::lexical_cast<std::string>(transform.getOrigin().z()) + "}\n";
	data += "  orientation: {x: " + boost::lexical_cast<std::string>(transform.getRotation().x()) + ", ";
	data += "y: " + boost::lexical_cast<std::string>(transform.getRotation().y()) + ", ";
	data += "z: " + boost::lexical_cast<std::string>(transform.getRotation().z()) + ", ";
	data += "w: " + boost::lexical_cast<std::string>(transform.getRotation().w()) + "}\n";
	// write inputted data into the file.
	outfile << data << std::endl;

	// close the opened file.
	outfile.close();

	ROS_INFO("The robot position is recored.");
	return 0;
}

#pragma once

#include <ros/ros.h>


namespace dr_util {

/// Get a parameter from the ROS parameter server.
template<typename T>
T get_param(ros::NodeHandle const & node, std::string const & name) {
	T value;
	if (!node.getParam(name, value)) {
		throw std::runtime_error(std::string("Failed to get parameter `") + name + "' (" + node.resolveName(name) + ").");
	}
	return value;
}

/// Get a parameter from the ROS parameter server.
template<typename T>
T get_param(ros::NodeHandle const & node, std::string const & name, T const & fallback) {
	T value;
	return node.getParam(name, value) ? value : fallback;
}

class node {
protected:
	/// The ROS node handle.
	ros::NodeHandle node_handle;

	/// Construct the node.
	node() : node_handle("~") {};

	/// Get a paramter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, an exception is thrown.
	 */
	template<typename T>
	T get_param(std::string const & name) {
		return dr_util::get_param<T>(node_handle, name);
	}

	/// Get a paramter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	T get_param(std::string const & name, T const & fallback) {
		return dr_util::get_param<T>(node_handle, name, fallback);
	}

public:
	/// Check if ROS thinks we should keep running.
	bool ok() {
		return node_handle.ok();
	}
};

}

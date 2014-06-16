#pragma once

#include <ros/ros.h>


namespace dr_util {

/// Get a parameter from the ROS parameter server.
template<typename T>
T getParam(ros::NodeHandle const & node, std::string const & name) {
	T value;
	if (!node.getParam(name, value)) {
		throw std::runtime_error(std::string("Failed to get parameter `") + name + "' (" + node.resolveName(name) + ").");
	}
	return value;
}

/// Get a parameter from the ROS parameter server.
template<typename T>
T getParam(ros::NodeHandle const & node, std::string const & name, T const & fallback) {
	T value;

	if (!node.getParam(name, value)) {
		printf("Failed to get parameter `%s' (%s). Using the default value.\n",name.c_str(), node.resolveName(name).c_str());
		value = fallback;
	}

	return value;
}

/// Get a list of parameters for the ROS parameter server.
template<typename T>
std::vector<T> getParamList(ros::NodeHandle const & node, std::string const & name){
	XmlRpc::XmlRpcValue list_xml;
	std::vector<T> values;
	if (!node.getParam(name,list_xml)) {
		throw std::runtime_error(std::string("Failed to get parameter `") + name + "' (" + node.resolveName(name) + ").");
	}
	else {
		if(list_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
			throw std::runtime_error(std::string("Failed to get parameter `") + name + "' (" + node.resolveName(name) + "). Invalid type.");
		for (int32_t i = 0; i < list_xml.size(); ++i) {
			values.push_back(static_cast<T>(list_xml[i]));
		}
	}
	return values;
}

template<typename T>
std::vector<T> getParamList(ros::NodeHandle const & node, std::string const & name, std::vector<T> const & fallback){
	XmlRpc::XmlRpcValue list_xml;
	std::vector<T> values;
	if (!node.getParam(name,list_xml)) {
		printf("Failed to get parameter `%s' (%s). Using the default value.\n",name.c_str(), node.resolveName(name).c_str());
		values = fallback;
	}
	else {
		if(list_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
			throw std::runtime_error(std::string("Failed to get parameter `") + name + "' (" + node.resolveName(name) + "). Invalid type.");
		for (int32_t i = 0; i < list_xml.size(); ++i) {
			values.push_back(static_cast<T>(list_xml[i]));
		}
	}
	return values;
}

class Node {
protected:
	/// The ROS node handle.
	ros::NodeHandle node_handle_;

	/// Construct the node.
	Node() : node_handle_("~") {};

	/// Get a parameter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, an exception is thrown.
	 */
	template<typename T>
	T getParam(std::string const & name) {
		return dr_util::getParam<T>(node_handle_, name);
	}

	/// Get a parameter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	T getParam(std::string const & name, T const & fallback) {
		return dr_util::getParam<T>(node_handle_, name, fallback);
	}

	/// Get a list of parameters from the ROS parameter server.
	/**
	 * If the parameters do not exist on the parameter server, an exception is thrown.
	 */
	template<typename T>
	std::vector<T> getParamList(std::string const & name) {
		return dr_util::getParamList<T>(node_handle_, name);
	}

	/// Get a list of parameters from the ROS parameter server.
	/**
	 * If the parameters do not exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	std::vector<T> getParamList(std::string const & name, T const & fallback) {
		return dr_util::getParamList<T>(node_handle_, name, fallback);
	}

public:
	/// Check if ROS thinks we should keep running.
	bool ok() {
		return node_handle_.ok();
	}
};

}

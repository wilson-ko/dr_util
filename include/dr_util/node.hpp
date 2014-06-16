#pragma once
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>


namespace dr_util {

/// Get a parameter from the ROS parameter server.
template<typename T>
T getParam(
	ros::NodeHandle const & node, ///< The node handle to use for parameter name resolution.
	std::string const & name      ///< The parameter to retrieve.
) {
	T value;
	if (!node.getParam(name, value)) {
		throw std::runtime_error(std::string("Failed to get parameter `") + name + "' (" + node.resolveName(name) + ").");
	}
	return value;
}

/// Get a parameter from the ROS parameter server.
template<typename T>
T getParam(
	ros::NodeHandle const & node, ///< The node handle to use for parameter name resolution.
	std::string const & name,     ///< The parameter to retrieve.
	T const & fallback,           ///< The fallback value to return if the parameter is not found.
	bool warn = true              ///< If true, log a warning when the parameter was not found.
) {
	T value;

	if (!node.getParam(name, value)) {
		if (warn) {
			ROS_WARN_STREAM("Failed to get parameter `" << name << "' (" << node.resolveName(name) << "). Using the default value.");
		}
		return fallback;
	}

	return value;
}

/// Get a vector from the ROS parameter server.
template<typename T>
std::vector<T> getParamList(
	ros::NodeHandle const & node, ///< The node handle to use for parameter name resolution.
	std::string const & name      ///< The parameter to retrieve.
) {
	return getParam<std::vector<T>>(node, name);
}

/// Get a vector from the ROS parameter server.
template<typename T>
std::vector<T> getParamList(
	ros::NodeHandle const & node,    ///< The node handle to use for parameter name resolution.
	std::string const & name,        ///< The parameter to retrieve.
	std::vector<T> const & fallback, ///< The fallback value to return if the parameter is not found.
	bool warn = false                ///< If true, log a warning when the parameter was not found.
) {
	return getParam<std::vector<T>>(node, name, fallback, warn);
}

/// A ROS node wrapper with some utility functions.
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

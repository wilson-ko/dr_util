#pragma once

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>


namespace dr {

/// Shorthand to get a ServiceEvent type for a service.
template<typename Service>
using ServiceEvent = ros::ServiceEvent<typename Service::Request, typename Service::Response>;

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

/// Get an array from the ROS parameter server.
template<typename T, std::size_t N>
std::array<T, N> getParamArray(
	ros::NodeHandle const & node,      ///< The node handle to use for parameter name resolution.
	std::string const & name           ///< The parameter to retrieve.
) {
	std::vector<T> value;
	if (!node.getParam(name, value)) throw std::runtime_error(std::string("Failed to get parameter `") + name + "' (" + node.resolveName(name) + ").");
	if (value.size() != N) throw std::runtime_error("Wrong number of elements in parameter `" + name + "' (" + node.resolveName(name) + ").");
	std::array<T, N> result;
	std::copy(value.begin(), value.end(), result.begin());
	return result;
}

/// Get an array from the ROS parameter server.
template<typename T, std::size_t N>
std::array<T, N> getParamArray(
	ros::NodeHandle const & node,      ///< The node handle to use for parameter name resolution.
	std::string const & name,          ///< The parameter to retrieve.
	std::array<T, N> const & fallback, ///< The fallback value to return if the parameter is not found.
	bool warn = false                  ///< If true, log a warning when the parameter was not found.
) {
	std::vector<T> value;
	if (!node.getParam(name, value)) {
		if (warn) ROS_WARN_STREAM("Failed to get parameter `" << name << "' (" << node.resolveName(name) << "). Using the default value.");
		return fallback;
	}
	if (value.size() != N) throw std::runtime_error("Wrong number of elements in parameter `" + name + "' (" + node.resolveName(name) + ").");
	std::array<T, N> result;
	std::copy(value.begin(), value.end(), result.begin());
	return result;
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
		return dr::getParam<T>(node_handle_, name);
	}

	/// Get a parameter from the ROS parameter server.
	/**
	 * If the parameter doesn't exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	T getParam(std::string const & name, T const & fallback) {
		return dr::getParam<T>(node_handle_, name, fallback);
	}

	/// Get a list of parameters from the ROS parameter server.
	/**
	 * If the parameters do not exist on the parameter server, an exception is thrown.
	 */
	template<typename T>
	std::vector<T> getParamList(std::string const & name) {
		return dr::getParamList<T>(node_handle_, name);
	}

	/// Get a list of parameters from the ROS parameter server.
	/**
	 * If the parameters do not exist on the parameter server, the fallback is returned.
	 */
	template<typename T>
	std::vector<T> getParamList(std::string const & name, T const & fallback) {
		return dr::getParamList<T>(node_handle_, name, fallback);
	}

public:
	/// Check if ROS thinks we should keep running.
	bool ok() {
		return node_handle_.ok();
	}
};

}

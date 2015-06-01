#pragma once

#include "xmlrpc.hpp"

#include <dr_log/dr_log.hpp>

#include <ros/param.h>
#include <ros/node_handle.h>

#include <stdexcept>


namespace dr {

/// Load a parameter from the ROS parameter service.
/**
 * Requires dr::ConvertXmlRpc<T> to be specialized for T.
 *
 * \throws Any exception thrown by dr::fromXmlRpc<T>.
 * \return True if the parameter was found, false otherwise.
 */
template<typename T>
bool loadParam(
	std::string const & key, ///< The key of the parameter to load.
	T & result               ///<[out] Output variable for the result.
) {
	XmlRpc::XmlRpcValue value;
	if (!ros::param::get(key, value)) return false;
	try {
		result = ConvertXmlRpc<T>::convert(value);
	} catch(std::exception const & e) {
		std::throw_with_nested(std::runtime_error("Failed to load parameter `" + key + "': " + e.what()));
	} catch (...) {
		std::throw_with_nested(std::runtime_error("Failed to load parameter `" + key + "'"));
	}
}

/// Get a parameter from the ROS parameter service.
/**
 * Requires dr::ConvertXmlRpc<T> to be specialized for T.
 *
 * \throws If the parameter can not be found or any exception thrown by dr::fromXmlRpc<T>.
 * \return The loaded parameter.
 */
template<typename T>
T getParam(
	std::string const & key ///< The key of the parameter to load.
) {
	XmlRpc::XmlRpcValue value;
	if (!ros::param::get(key, value)) throw std::runtime_error("ROS parameter not found: " + key);
	try {
		return ConvertXmlRpc<T>::convert(value);
	} catch(std::exception const & e) {
		std::throw_with_nested(std::runtime_error("Failed to load parameter `" + key + "': " + e.what()));
	} catch (...) {
		std::throw_with_nested(std::runtime_error("Failed to load parameter `" + key + "'"));
	}
}

/// Get a parameter from the ROS parameter service or a fallback value.
/**
 * Requires dr::ConvertXmlRpc<T> to be specialized for T.
 *
 * \throws Any exception thrown by dr::fromXmlRpc<T>.
 * \return The loaded parameter or the fallback value if the parameter was not found.
 */
template<typename T>
T getParam(
	std::string const & key, ///< The key of the parameter to load.
	T const & fallback,      ///< The fallback value to return if the parameter can not be found.
	bool warn = true         ///< If true, log a warning when the fallback default value is used.
) {
	XmlRpc::XmlRpcValue value;
	if (!ros::param::get(key, value)) {
		if (warn) DR_WARN("Failed to find ROS parameter: " << key << ". Using fallback value.");
		return fallback;
	}

	try {
		return ConvertXmlRpc<T>::convert(value);
	} catch(std::exception const & e) {
		std::throw_with_nested(std::runtime_error("Failed to load parameter `" + key + "': " + e.what()));
	} catch (...) {
		std::throw_with_nested(std::runtime_error("Failed to load parameter `" + key + "'"));
	}
}

template<typename T>
bool loadParam(ros::NodeHandle const & node, std::string const & key, T & result) {
	return loadParam(node.resolveName(key), result);
}

template<typename T>
T getParam(ros::NodeHandle const & node, std::string const & key) {
	return getParam<T>(node.resolveName(key));
}

template<typename T>
T getParam(ros::NodeHandle const & node, std::string const & key, T const & fallback, bool warn = true) {
	return getParam<T>(node.resolveName(key), fallback, warn);
}

}

#include <string>

#include <ros/ros.h>

namespace dr {

class ServiceError : std::runtime_error {
	using std::runtime_error::runtime_error;
};

/// Wrapper around ros::ServiceClient for easy presistent, reconnecting services.
template<typename Service>
class ServiceClient {
	using Request  = typename Service::Request;
	using Response = typename Service::Response;

	/// The node to use for connecting to services.
	ros::NodeHandle * node_ = nullptr;

	/// The internal service client.
	ros::ServiceClient client_;

	/// The name of the service.
	std::string name_;

public:
	/// Construct a service client without connecting it to a service.
	ServiceClient() {}

	/// Construct a service client and connect it to a service.
	ServiceClient(ros::NodeHandle & node, std::string const & name, ros::Duration timeout = ros::Duration(-1), bool verbose = true) : ServiceClient(node) {
		connect(node, name, timeout, verbose);
	}

	/// Connect to a service by name.
	bool connect(ros::NodeHandle & node, std::string name, ros::Duration const & timeout = ros::Duration(-1), bool verbose = true) {
		node_   = &node;
		name_   = name;
		client_ = node_->serviceClient<Request, Response>(name_, true);
		return wait(timeout, verbose);
	}

	/// Reconnect to the current service if the connection was dropped.
	/**
	 * Does nothing if the service is still connected.
	 */
	bool reconnect(ros::Duration const & timeout = ros::Duration(-1), bool verbose = true) {
		if (client_.isValid()) return true;
		if (verbose) ROS_WARN_STREAM("Lost connection to service `" + name_ + "'. Reconnecting.");
		client_ = node_->serviceClient<Request, Response>(name_, true);
		return wait(timeout, verbose);
	}

	/// Wait for the service to connect.
	bool wait(ros::Duration const & timeout = ros::Duration(-1), bool verbose = true) {
		if (verbose) ROS_INFO_STREAM("Waiting for service `" << name_ << "'.");
		return client_.waitForExistence(timeout);
	}

	/// Call the service and return the response.
	/**
	 * \throws ServiceError when the service call fails.
	 */
	Response operator() (Request const & request, bool reconnect = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		Response response;
		if (reconnect) this->reconnect(timeout, verbose);
		if (!client_.call(request, response)) throw ServiceError("Failed to call service `" + name_ + "'.");
		return response;
	}

	/// Call the service.
	bool operator() (Request const & request, Response & response, bool reconnect = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		if (reconnect) this->reconnect(timeout, verbose);
		return client_.call(request, response);
	}

	/// Call the service.
	bool operator() (Service & service, bool reconnect = true, ros::Duration timeout = ros::Duration(-1), bool verbose = true) {
		if (reconnect) this->reconnect(timeout, verbose);
		return client_.call(service);
	}
};

}

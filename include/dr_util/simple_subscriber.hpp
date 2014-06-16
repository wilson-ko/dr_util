#pragma once

#include <string>

#include <ros/ros.h>

namespace dr {

/// Simple subscriber that remembers the last message received.
template<typename Message>
class SimpleSubscriber {
private:
	/// The cached message.
	Message message_;

	/// The internal subscriber used.
	ros::Subscriber subscriber_;

public:
	/// Construct the subscriber.
	SimpleSubscriber(
		ros::NodeHandle & node,                  ///< The node handle to use for name resolution.
		std::string const & topic,               ///< The name of the topic to subscribe to.
		uint32_t queue_size = 10,                ///< The queue size for the wrapper subscriber.
		ros::TransportHints transport_hints = {} ///< The transport hints for the wrapper subscriber.
	) {
		subscriber_ = node.subscribe(topic, queue_size, &SimpleSubscriber::onMessage, this, transport_hints);
	};

	/// Get the last message received.
	Message const & message() const { return message_; };

private:
	/// Handle received messages.
	void onMessage(Message const & message) {
		message_ = message;
	}
};

}

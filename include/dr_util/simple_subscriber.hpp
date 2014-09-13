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

	/// The topic name.
	std::string topic_name_ = "";

	/// True if a message has been received.
	bool message_received_ = false;

	/// Signal to notify the wait function it can return.
	bool wait_done_;

public:
	SimpleSubscriber(){
	}
	/// Construct the subscriber.
	SimpleSubscriber(
		ros::NodeHandle & node,                  ///< The node handle to use for name resolution.
		std::string const & topic,               ///< The name of the topic to subscribe to.
		uint32_t queue_size = 10,                ///< The queue size for the wrapper subscriber.
		ros::TransportHints transport_hints = {} ///< The transport hints for the wrapper subscriber.
	) {
		initialize(node, topic,queue_size,transport_hints);
	}

	void initialize(ros::NodeHandle & node,      ///< The node handle to use for name resolution.
		std::string const & topic,               ///< The name of the topic to subscribe to.
		uint32_t queue_size = 10,                ///< The queue size for the wrapper subscriber.
		ros::TransportHints transport_hints = {} ///< The transport hints for the wrapper subscriber.
		){
		subscriber_ = node.subscribe(topic, queue_size, &SimpleSubscriber::onMessage, this, transport_hints);
		topic_name_ = topic;
	}

	/// Get the last message received.
	Message const & message() const { 
		if(!message_received_)
			ROS_ERROR("No messages has been received to the topic %s, but the message is requested.",topic_name_.c_str());
		if(topic_name_ == "")
			ROS_ERROR("Subscriber not initialized, but the message is requested.");
		return message_;
	}

	/// Check if the subscriber has a message.
	bool hasMessage() const {
		return message_received_;
	}

	/// Wait for a message to a arrive.
	bool wait(ros::Duration timeout = ros::Duration(0), ros::Rate rate = ros::Rate(10)) {
		wait_done_ = false;
		ros::Time start = ros::Time::now();

		while (timeout == ros::Duration(0) || (ros::Time::now() - start) < timeout) {
			ros::spinOnce();
			if (wait_done_) return true;
			rate.sleep();
		}

		return wait_done_;
	}

private:
	/// Handle received messages.
	void onMessage(Message const & message) {
		message_          = message;
		message_received_ = true;
		wait_done_        = true;
	}
};

}

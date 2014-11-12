#pragma once

#include <functional>
#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>

namespace dr {

/// Simple subscriber that remembers the last message received.
template<typename Message>
class Subscriber {
private:
	/// The cached message.
	Message message_;

	ros::Time time_{0, 0};

	/// The internal subscriber used.
	ros::Subscriber subscriber_;

	/// The topic name.
	std::string topic_name_ = "";

	/// True if a message has been received.
	bool message_received_ = false;

	/// Signal to notify the wait function it can return.
	bool wait_done_;

	/// All synchronous wait operations started before this time should cancel themselves.
	ros::Time cancel_before_{0, 0};

	/// Vector of callbacks to invoke on the next message.
	std::vector<std::function<void (Message const &)>> waiters_;

	/// Mutex to obtain when accesing the message.
	std::mutex message_mutex_;

	/// Mutex to obtain when accesing the callback list.
	std::mutex callback_mutex_;

public:
	Subscriber() {}

	/// Construct the subscriber.
	Subscriber(
		ros::NodeHandle & node,                  ///< The node handle to use for name resolution.
		std::string const & topic,               ///< The name of the topic to subscribe to.
		uint32_t queue_size = 10,                ///< The queue size for the wrapper subscriber.
		ros::TransportHints transport_hints = {} ///< The transport hints for the wrapper subscriber.
	) {
		initialize(node, topic,queue_size,transport_hints);
	}

	/// Initialize the subscriber.
	void initialize(ros::NodeHandle & node,      ///< The node handle to use for name resolution.
		std::string const & topic,               ///< The name of the topic to subscribe to.
		uint32_t queue_size = 10,                ///< The queue size for the wrapper subscriber.
		ros::TransportHints transport_hints = {} ///< The transport hints for the wrapper subscriber.
	) {
		subscriber_ = node.subscribe(topic, queue_size, &Subscriber::onMessage, this, transport_hints);
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
			if (ros::Time::now() <= cancel_before_) return false;
			rate.sleep();
		}

		return wait_done_;
	}

	/// Register a callback to be called once when the next message arrives.
	void asyncWait(std::function<void (Message const &)> callback) {
		std::lock_guard<std::mutex> lock(callback_mutex_);
		waiters_.push_back(callback);
	};

	/// Register a callback to be called once when the next message arrives.
	template<typename F>
	void asyncWait(
		std::function<void (Message const &, ros::Time const &)> callback, ///< Callback to be invoked when the condition becomes true.
		F const & condition                                                ///< Condition to check. Will be called as condition(message, time);
	) {
		checkCondition(message(), time_, callback, condition);
	}

	/// Register a callback to be called once when the next message arrives.
	template<typename F>
	void asyncWait(
		std::function<void (Message const &, ros::Time const &)> callback, ///< Callback to be invoked when the condition becomes true.
		F const & condition,                                               ///< Condition to check. Will be called as condition(message, time);
		ros::Duration const & grace_period                                 ///< Grace period to wait before checking the condition.
	) {
		ros::Time grace_end  = ros::Time::now() + grace_period;
		checkCondition(message(), time_, callback, [condition, grace_end](Message const & message, ros::Time const & time) {
			if (time < grace_end) return false;
			return condition(message, time);
		});
	}

	/// Cancel all pending wait operations (synchronous and asynchronous).
	void cancel() {
		std::lock_guard<std::mutex> lock(callback_mutex_);
		cancel_before_ = ros::Time::now();
		waiters_.clear();
	}

private:
	/// Handle received messages.
	void onMessage(Message const & message) {
		std::lock_guard<std::mutex> lock(message_mutex_);

		message_          = message;
		time_             = ros::Time::now();
		message_received_ = true;
		wait_done_        = true;

		{
			std::lock_guard<std::mutex> lock(callback_mutex_);
			for (auto const & callback : waiters_) callback(message);
		}
	}

	/// Check a condition for a message and time.
	/**
	 * If the condition is true, a callback is invoked.
	 * If the condition is false, the condition is requeued.
	 */
	template<typename F>
	bool checkCondition(
		Message const & message,                                            ///< The message to check the condition against.
		ros::Time const & time,                                             ///< The time when the message arrived.
		std::function<void (Message const &, ros::Time const &)> callback,  ///< The callback to invoke if the condition is true.
		F const & condition                                                 ///< The condition to check. Invoked as condition(message, time).
	) {
		if (!condition(message, time)) {
			waiters_.push_back([this, callback, condition] (Message const & message, ros::Time const & time) {
				checkCondition(message, time, callback, condition);
			});
			return false;
		}
		callback(message, time);
		return true;
	}
};

}

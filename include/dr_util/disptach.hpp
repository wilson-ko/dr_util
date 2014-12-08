#pragma once

#include <utility>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/callback_queue_interface.h>

namespace dr {

template<typename F>
class RosCallbackWrapper : public ros::CallbackInterface {
	F f;

public:
	RosCallbackWrapper(F && f) : f(std::forward<F>(f)) {};

	virtual CallResult call() override {
		f();
		return CallResult::Success;
	}
};

/// Dispatch a callback for later invocation.
template<typename F>
void dispatch(ros::NodeHandle node, F && f) {
	node.getCallbackQueue()->addCallback(boost::make_shared<RosCallbackWrapper<F>>(std::forward<F>(f)));
}

}

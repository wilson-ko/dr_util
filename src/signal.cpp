#include <iostream>
#include <cstdio>

#include <signal.h>

#include <ros/ros.h>

#include "signal.hpp"

namespace dr {

namespace {
	struct sigaction sigaction_data = {};

	void handleSigInt(int sig, siginfo_t * siginfo, void * context) {
		(void) sig;
		(void) context;
		(void) siginfo;
		std::fclose(stdin);
		ros::shutdown();
	}

}

int fixSigInt() {
	sigaction_data.sa_flags = SA_SIGINFO;
	sigaction_data.sa_sigaction = &handleSigInt;
	return sigaction(SIGINT, &sigaction_data, nullptr);
}

}

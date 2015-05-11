#include "timestamp.hpp"
#include <ctime>

void getTimeString(std::string & output) {
	time_t now = time(0);
	tm * ltm = localtime(&now);
	char mbstr[100];
	std::strftime(mbstr, sizeof(mbstr), "%F_%H-%M-%S", ltm);
	output = mbstr;
}

std::string getTimeString() {
	std::string time;
	getTimeString(time);
	return time;
}

std::string getDateString() {
	time_t now = time(0);
	tm * ltm = localtime(&now);
	char mbstr[100];
	std::strftime(mbstr, sizeof(mbstr), "%F", ltm);
	return mbstr;
}

#pragma once
#include <string>

namespace dr {
	/// Creates a string with the current time.
	void getTimeString(std::string & output);

	/// Creates a string with the current time.
	std::string getTimeString();

	/// Creates a string with the current date.
	std::string getDateString();
}

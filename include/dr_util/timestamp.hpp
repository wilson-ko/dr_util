#pragma once
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>

namespace dr {

	// Get a string representing a posix ptime in the specified format
	std::string formatTime(
		boost::posix_time::ptime timestamp,
		std::string const & format
	);

	/// Get a string representing the time according to a specified format.
	std::string getTimeString(
		std::string const & format ///< A boost::date_time compatible format string.
	);

	/// Creates a string with the current time.
	void getTimeString(std::string & output);

	/// Creates a string with the current time.
	std::string getTimeString();

	/// Creates a string with the current date.
	std::string getDateString();
}

#include "resource.hpp"

#include <ros/package.h>

#include <algorithm>
#include <stdexcept>

namespace dr {

/// Convert a ROS package URL or a regular file URL to a file path.
std::string rosUrlToPath(std::string const & url) {
	std::string const scheme_seperator = "://";
	auto scheme_end = std::search(url.begin(), url.end(), scheme_seperator.begin(), scheme_seperator.end());

	// No scheme? Not really an URL, lets hope it already was a file path.
	// TODO: Throw exception instead?
	if (scheme_end == url.end()) return url;

	std::string scheme(url.begin(), scheme_end);
	auto authority_start = scheme_end + 3;
	auto path_start      = std::find(authority_start, url.end(), '/');

	std::string authority(authority_start, path_start);
	std::string path(path_start, url.end());

	if (scheme == "package") {
		return ros::package::getPath(authority) + path;
	} else if (scheme == "file") {
		if (authority.size()) throw std::runtime_error("Tried to convert non-local file URL to file path: " + url);
		return path;
	}

	throw std::runtime_error("Unsupported URL scheme `" + scheme + "' in URL: " + url);

}

}

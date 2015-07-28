#pragma once
#include <string>

namespace dr {

/// Convert a ROS package URL or a regular file URL to a file path.
/**
 * \throws if an unsupported URL scheme is encountered.
 */
std::string rosUrlToPath(std::string const & url);

}

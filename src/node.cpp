#include "node.hpp"
#include "file_system.hpp"

#include <boost/algorithm/string.hpp>
#include <dr_log/dr_log.hpp>

namespace dr {

namespace {

std::string namespaceToName(std::string const & name) {
	std::string result = name;
	if (boost::algorithm::starts_with(result, "/")) result.erase(0, 1);
	boost::algorithm::replace_all(result, "/", ".");
	return result;
}

}

Node::Node() : node_handle_("~") {
	std::string node_name = namespaceToName(node_handle_.getNamespace());
	run_prefix_  = searchParam<std::string>("run_prefix", getHomeDirectory() + "/.ros/log");
	node_prefix_ = run_prefix_ + "/" + node_name;
	std::string log_file = node_prefix_ + "/" + node_name + ".log";

	setupLogging(log_file, node_name);
};

}

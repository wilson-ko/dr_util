#include "node.hpp"
#include "file_system.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/date_time/time_facet.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <sstream>

namespace dr {

namespace {

std::string namespaceToName(std::string const & name) {
	std::string result = name;
	if (boost::algorithm::starts_with(result, "/")) result.erase(0, 1);
	boost::algorithm::replace_all(result, "/", ".");
	return result;
}

std::string formatTime(boost::posix_time::ptime time, std::string const & format) {
	using namespace boost::posix_time;

	std::stringstream buffer;
	std::locale locale(buffer.getloc(), new time_facet(format.c_str()));
	buffer.imbue(locale);
	buffer << time;
	return buffer.str();
}

}

Node::Node() : node_handle_("~") {
	boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

	std::string node_name = namespaceToName(node_handle_.getNamespace());
	std::string fallback  = getHomeDirectory() + "/.ros/run/" + formatTime(now, "%Y-%m-%d/%H-%M-%S");
	run_prefix_           = searchParam<std::string>("run_prefix", fallback);
	node_prefix_          = run_prefix_  + "/" + node_name;
	std::string log_file  = node_prefix_ + "/" + node_name + ".log";

	setupLogging(log_file, node_name);
};

std::string Node::runPrefix() {
	return run_prefix_;
}

std::string Node::nodePrefix() {
	return node_prefix_;
}

}

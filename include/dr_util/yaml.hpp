#pragma once

#include <string>
#include <vector>
#include <boost/array.hpp>

namespace dr {
	template <typename T>
	std::string toYaml(std::vector<T> array, std::string name) {
		std::string output = name + ": [";
		for (T const & item: array) {
			output += std::to_string(item) + ", ";
		}
		return output.substr(0, output.size() - 2) + "]";
	}

	template <typename T, size_t S>
	std::string toYaml(boost::array<T, S> array, std::string name) {
		std::string output = name + ": [";
		for (T const & item: array) {
			output += std::to_string(item) + ", ";
		}
		return output.substr(0, output.size() - 2) + "]";
	}

}

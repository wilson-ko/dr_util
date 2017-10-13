#include "environment.hpp"
#include "expand.hpp"

#include <regex>
#include <iostream>

namespace dr {

using namespace std::string_literals;

namespace {
	template<typename Key, typename Value>
	Value const & lookupOr(std::map<Key, Value> const & map, Key const & key, Value const & fallback) {
		auto iterator = map.find(key);
		if (iterator == map.end()) return fallback;
		return iterator->second;
	}
}

std::string expandVariables(std::string const & source, std::map<std::string, std::string> const & variables) {
	static std::regex pattern{"\\$\\{([a-zA-Z0-9_]+)\\}|\\$([a-zA-Z0-9_]+)"};

	std::string::const_iterator start = source.begin();
	std::string result;
	result.reserve(source.size() * 1.2  + 30);

	for (std::sregex_iterator i{source.begin(), source.end(), pattern}; i != std::sregex_iterator{}; ++i) {
		std::smatch const & match = *i;
		std::string key = match[1].matched ? match[1].str() : match[2].str();
		result.append(start, match[0].first);
		result.append(lookupOr(variables, key, ""s));
		start = match[0].second;
	}

	result.append(start, source.end());
	return result;
}

std::string expandEnvironment(std::string const & source) {
	return expandVariables(source, getEnvironment());
}

}

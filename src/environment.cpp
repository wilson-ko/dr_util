#include <environment.hpp>

#include <cstring>

extern "C" char **environ;

namespace dr {

	std::map<std::string, std::string> getEnvironment() {
		std::map<std::string, std::string> result;
		if (environ == nullptr) { return {}; }

		for (char ** item = environ; *item != nullptr; ++item) {
			// Find seperator ('=')
			char * sep = std::strchr(*item, '=');
			if (!static_cast<bool>(sep)) { continue; }
			char * end = std::strchr(sep + 1, '\0');
			if (!static_cast<bool>(end)) { continue; } // This should never happend.

			// Insert result in the map.
			result.insert(std::make_pair(std::string{*item, sep}, std::string{sep + 1, end}));
		};
		return result;
	}

} //namespace dr

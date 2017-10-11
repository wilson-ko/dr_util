#include <environment.hpp>

extern "C" char **environ;

namespace dr {

	namespace {
		char * findCstr(char * source, char target) {
			char * result = source;
			while (*result && *result != target) ++result;
			return result;
		}
	}

	std::map<std::string, std::string> getEnvironment() {
		std::map<std::string, std::string> result;
		if (environ == nullptr) return {};

		for (char ** item = environ; *item != nullptr; ++item) {
			// Find seperator ('=')
			char * sep = findCstr(*item, '='); if (*sep != '=') continue;
			char * end = findCstr(sep + 1, '\0');

			// Insert result in the map.
			result.insert(std::make_pair(std::string{*item, sep}, std::string{sep + 1, end}));
		};
		return result;
	}

}

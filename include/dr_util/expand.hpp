#include <map>
#include <string>

namespace dr {

/// Expand variables in a string.
/**
 * A variable can be written as $NAME or ${NAME}.
 */
std::string expandVariables(std::string const & source, std::map<std::string, std::string> const & variables);

/// Expand environment variables in a string.
/**
 * A variable can be written as $NAME or ${NAME}.
 */
std::string expandEnvironment(std::string const & source);

}

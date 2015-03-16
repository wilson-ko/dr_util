#pragma once
#include <iostream>

namespace dr {

	bool createDirectory(std::string const & path_string);

	bool createParentDirectory(std::string const & filename_string);

	std::string getHomeDirectory();

}

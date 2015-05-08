#pragma once
#include <iostream>
#include <vector>

namespace dr {

	bool createDirectory(std::string const & path);

	bool createParentDirectory(std::string const & filename);

	std::string getHomeDirectory();

	std::vector<std::string> getFilesInDirectoryRecursive(std::string const & directory);
	std::vector<std::string> getFilesInDirectoryRecursive(std::string const & directory, std::string const & suffix);

	/// Returns true if a given string has a given suffix.
	bool hasSuffix(
		const std::string & string,                 ///< Input string
		const std::string & suffix                  ///< Input suffix
	);

}

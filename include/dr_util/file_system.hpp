#pragma once
#include <iostream>
#include <vector>

namespace dr {

	bool createDirectory(std::string const & path);

	bool createParentDirectory(std::string const & filename);

	std::string getHomeDirectory(std::string const & fallback = "./");

	std::vector<std::string> getFilesInDirectoryRecursive(std::string const & directory);
	std::vector<std::string> getFilesInDirectoryRecursive(std::string const & directory, std::string const & suffix);

	/// Gets a list of all file names from a given directory.
	void getFilesInDirectory(
		const std::string & directory,              ///< Input directory path
		std::vector<std::string> & file_names       ///< Vector of filenames contained in the directory
	);

	/// Gets a list of all file names from a given directory.
	void getFilesInDirectory(
		const std::string & directory,
		std::vector<std::string> & file_names,
		std::string suffix
	);

	/// Gets a list of all file names from a given directory.
	std::vector<std::string> getFilesInDirectory(
		const std::string & directory,
		std::string suffix
	);

	/// Returns true if a given string has a given suffix.
	bool hasSuffix(
		const std::string & string,                 ///< Input string
		const std::string & suffix                  ///< Input suffix
	);

}

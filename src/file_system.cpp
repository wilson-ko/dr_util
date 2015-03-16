#include <file_system.hpp>
#include <boost/filesystem.hpp>

namespace dr {

bool createDirectory(std::string const & path_string){

	boost::filesystem::path path(path_string);

	// Return immediately if the directory already exists.
	if (boost::filesystem::is_directory(path)){
		std::cout << path.c_str() << " does not need to be created, since it already exists." << std::endl;
		return true;
	}

	// Create parent directory if parent does not exist.
	if (!boost::filesystem::is_directory(path.parent_path())){
		std::cout << "Need to create parent directory (" << path.parent_path().c_str() << "as well." << std::endl;
		createDirectory(path.parent_path().c_str());
	}

	// Create directory
	if (boost::filesystem::create_directory(path)){
		std::cout << "Successfully created" << path_string << std::endl;
	}
	else
		std::cout << "Unable to create directory " << path_string << std::endl;

	return true;

}

}

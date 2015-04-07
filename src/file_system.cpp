#include <file_system.hpp>
#include <boost/filesystem.hpp>

namespace dr {

bool createDirectory(std::string const & path_string){

	boost::filesystem::path path(path_string);

	// Return immediately if the directory already exists.
	if (boost::filesystem::is_directory(path)){
		std::cout << "'" << path.c_str() << "' does not need to be created, since it already exists." << std::endl;
		return true;
	}

	// Create parent directory if parent does not exist.
	if (!boost::filesystem::is_directory(path.parent_path())){
		std::cout << "Need to create parent directory '" << path.parent_path().c_str() << "' as well." << std::endl;
		createDirectory(path.parent_path().c_str());
	}

	// Create directory
	if (boost::filesystem::create_directory(path)){
		std::cout << "Successfully created '" << path_string << "'." << std::endl;
	}
	else
		std::cout << "Unable to create directory '" << path_string << "'." << std::endl;

	return true;

}

bool createParentDirectory(std::string const & filename_string){
	boost::filesystem::path filename(filename_string);

	// Create the directory if it does not exist.
	if ( !boost::filesystem::is_directory(filename.parent_path()) )
		createDirectory(filename.parent_path().c_str());

	return true;
}

std::vector<std::string> getFilesInDirectoryRecursive(std::string const & directory){
	std::vector<std::string> files;
	boost::filesystem::path start_at(directory);
	boost::filesystem::recursive_directory_iterator iterator(start_at);
	while(iterator != boost::filesystem::recursive_directory_iterator()){ // Iterate over all directories recursively.
		if ( boost::filesystem::is_regular_file(iterator->path()) ) // Only return file names, not names of the directories.
			files.push_back(iterator->path().string());
		++iterator;
	}
	return files;
}

std::vector<std::string> getFilesInDirectoryRecursive(std::string const & directory, std::string const & suffix){
	std::vector<std::string> all_files = getFilesInDirectoryRecursive(directory);
	std::vector<std::string> subset_files;
	for (size_t i=0; i<all_files.size(); i++){
		if (hasSuffix(all_files.at(i), suffix)){
			subset_files.push_back(all_files.at(i));
		}
	}
	return subset_files;
}

bool hasSuffix(const std::string & string, const std::string & suffix) {
	return string.size() >= suffix.size() && !string.compare(string.size() - suffix.size(), suffix.size(), suffix);
}

std::string getHomeDirectory(){
	const char *homedir;
	if ((homedir = getenv("HOME")) == NULL){
		std::cout << "Unable to get home directory as text string. Using working directory instead of home directory." << std::endl;
		return "./";
	}
	return homedir;
}

}

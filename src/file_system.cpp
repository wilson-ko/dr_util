#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <dirent.h>

#include <file_system.hpp>
#include <boost/filesystem.hpp>

namespace dr {

bool createDirectory(std::string const & path){
	boost::system::error_code error;
	return boost::filesystem::create_directories(path, error);
}

bool createParentDirectory(std::string const & filename){
	return createDirectory(boost::filesystem::path(filename).parent_path().c_str());
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

void getFilesInDirectory(const std::string & directory, std::vector<std::string> & file_names) {
	file_names.clear();
	DIR * dir;
	const char * c = directory.c_str();
	dir = opendir(c);
	dirent * pdir;
	while ((pdir = readdir(dir))) {
		file_names.push_back(pdir->d_name);
	}
	closedir(dir);
	std::sort(file_names.begin(), file_names.end());
}

void getFilesInDirectory(
	const std::string & directory, std::vector<std::string> & file_names, std::string suffix
) {
	file_names.clear();
	std::vector<std::string> all_file_names;
	getFilesInDirectory(directory, all_file_names);
	for (size_t i = 0; i < all_file_names.size(); i++) {
			if (hasSuffix(all_file_names.at(i), suffix)) {
				file_names.push_back(all_file_names.at(i));
			}
	}
}

std::vector<std::string> getFilesInDirectory(const std::string & directory, std::string suffix) {
	std::vector<std::string> file_names;
	getFilesInDirectory(directory, file_names, suffix);
	return file_names;
}

bool hasSuffix(const std::string & string, const std::string & suffix) {
	return string.size() >= suffix.size() && !string.compare(string.size() - suffix.size(), suffix.size(), suffix);
}

std::string getHomeDirectory(std::string const & fallback) {
	char const * homedir = std::getenv("HOME");
	if (homedir) return homedir;

	std::vector<std::uint8_t> buffer(256);
	uid_t uid = getuid();
	passwd info;
	passwd * result;

	while (true) {
		errno = 0;
		int error = getpwuid_r(uid, &info, reinterpret_cast<char *>(buffer.data()), buffer.size(), &result);

		if (!error) {
			return result != nullptr ? info.pw_dir : fallback;
		} else if (errno != ERANGE) {
			break;
		} else if (buffer.size() >= 1024) {
			break;
		} else {
			buffer.resize(buffer.size() * 2);
		}
	}

	return fallback;
}

}

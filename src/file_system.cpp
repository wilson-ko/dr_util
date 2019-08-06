#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <dirent.h>

#include <file_system.hpp>
#include <boost/filesystem.hpp>

namespace dr {

constexpr std::size_t BUFFER_SIZE = 256;
constexpr std::size_t BUFFER_THRESHOLD = 1024;

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
		if ( boost::filesystem::is_regular_file(iterator->path()) ){ // Only return file names, not names of the directories.
			files.push_back(iterator->path().string());
		}
		++iterator;
	}
	return files;
}

std::vector<std::string> getFilesInDirectoryRecursive(std::string const & directory, std::string const & suffix){
	std::vector<std::string> all_files = getFilesInDirectoryRecursive(directory);
	std::vector<std::string> subset_files;
	for (auto const & file : all_files) {
		if (hasSuffix(file, suffix)){
			subset_files.push_back(file);
		}
	}
	return subset_files;
}

bool hasSuffix(const std::string & string, const std::string & suffix) {
	return string.size() >= suffix.size() && !static_cast<bool>(string.compare(string.size() - suffix.size(), suffix.size(), suffix));
}

std::string getHomeDirectory(std::string const & fallback) {
	char const * homedir = std::getenv("HOME");
	bool homedir_empty = static_cast<bool>(homedir);
	if (homedir_empty) { return homedir; }

	std::vector<std::uint8_t> buffer(BUFFER_SIZE);
	uid_t uid = getuid();
	passwd info{};
	passwd * result;

	while (true) {
		errno = 0;
		int error = getpwuid_r(uid, &info, reinterpret_cast<char *>(buffer.data()), buffer.size(), &result);

		if (!static_cast<bool>(error)) {
			return result != nullptr ? info.pw_dir : fallback;
		}

		if (errno != ERANGE) {
			break;
		}

		if (buffer.size() >= BUFFER_THRESHOLD) {
			break;
		}

		buffer.resize(buffer.size() * 2);
	}

	return fallback;
}

} //namespace dr

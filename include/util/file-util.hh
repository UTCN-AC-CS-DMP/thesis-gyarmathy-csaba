#ifndef FILE_UTIL_HH
#define FILE_UTIL_HH

#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

class FileUtil {
 private:
  static std::string getFileOrDir(bool is_file = true);

 public:
  static std::string getSingleFileAbsPath();
  static std::string getDirectoryAbsPath();
  static std::vector<std::string> getAllFilesInDirectory();
};

#endif  // FILE_UTIL_HH

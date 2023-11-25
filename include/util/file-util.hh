#ifndef FILE_UTIL_HH
#define FILE_UTIL_HH

#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace utcn::ip {
class FileUtil {
 private:
  static std::string getFileOrDir(bool isFile = true);

 public:
  static std::string getSingleFileAbsPath();
  static std::string getDirectoryAbsPath();
  static std::vector<std::string> getAllFilesInDirectory();
};
}  // namespace utcn::ip

#endif  // FILE_UTIL_HH

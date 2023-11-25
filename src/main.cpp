#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "file-util.hh"

using fileutil = utcn::ip::FileUtil;

int main() {
  const std::string abs_image_path = fileutil::getSingleFileAbsPath();
  if (!abs_image_path.empty()) {
    const cv::Mat src = cv::imread(abs_image_path);
    cv::imshow("image", src);
    cv::waitKey();
  }

  return 0;
}
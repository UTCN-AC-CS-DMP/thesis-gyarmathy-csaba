#ifndef IMAGE_UTIL_HH
#define IMAGE_UTIL_HH

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class ImageUtil {
 public:
  static void resize_img(const cv::Mat &src, cv::Mat &dst, int maxSize,
                         bool interpolate);
  static bool is_inside(const cv::Mat &img, int i, int j);
};

#endif  // IMAGE_UTIL_HH

#include "image-util.hh"

void ImageUtil::resize_img(const cv::Mat& src, cv::Mat& dst, int maxSize,
                           bool interpolate) {
  double ratio = 1;
  const double w = src.cols;
  const double h = src.rows;
  if (w > h)
    ratio = w / static_cast<double>(maxSize);
  else
    ratio = h / static_cast<double>(maxSize);
  const int nw = static_cast<int>(w / ratio);
  const int nh = static_cast<int>(h / ratio);
  const cv::Size sz(nw, nh);
  if (interpolate)
    cv::resize(src, dst, sz);
  else
    resize(src, dst, sz, 0, 0, cv::INTER_NEAREST);
}

bool ImageUtil::is_inside(const cv::Mat& img, int i, int j) {
  return i >= 0 && i < img.rows && j >= 0 && j < img.cols;
}
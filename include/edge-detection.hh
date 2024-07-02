#ifndef EDGE_DETECTION_HH
#define EDGE_DETECTION_HH

#include <cmath>
#include <cstdint>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "image-util.hh"

/**
 * @brief Class for edge detection related operations
 */
class EdgeDetection {
 private:
  // π as a constant
  const double PI_20_PREC =
      3.141592653589793238462643383279502884197169399375105820974944;

  // Next direction to follow while tracing the contour
  const std::vector<int> y_chain_neighbours = {0, -1, -1, -1, 0, 1, 1, 1};
  const std::vector<int> x_chain_neighbours = {1, 1, 0, -1, -1, -1, 0, 1};

  // Neighboring pixels for labeling connected holes
  const std::vector<int> y_neighbours = {-1, -1, -1, 0, 0, 1, 1, 1};
  const std::vector<int> x_neighbours = {-1, 0, 1, -1, 1, -1, 0, 1};

  int total_holes;
  cv::Mat hole_labels;
  cv::Mat contour;
  std::vector<cv::Point> contour_points;
  std::vector<std::vector<cv::Point>> holes;

  const float epsilon = 1e-6;

 public:
  std::vector<std::pair<int, int>> calculate_perimeter(
      const cv::Mat_<uint8_t>& img);

  void fill_label(const cv::Mat& source, int x, int y);

  std::vector<cv::Point> trace_hole(const cv::Mat& source, int x, int y);

  void trace_contour(const cv::Mat& source, int x, int y);

  void draw_contour(const cv::Mat& source);

  void contour_wrapper(cv::Mat& source);

  void generate_gcode_holes(const std::string& filename, const cv::Mat& source,
                            float scale, float zHeight, float feedRate);

  cv::Mat canny(const cv::Mat& img);

  cv::Mat_<float> convolution(cv::Mat_<uint8_t>& img, cv::Mat_<float>& H);

  cv::Mat_<uint8_t> canny_edge_detection(cv::Mat_<uint8_t>& img);

  void generate_canny_gcode(const std::string& filename,
                            const cv::Mat_<uint8_t>& edgeMap, float scale,
                            float zHeight, float feedRate);

  bool approx_equal(float a, float b);
  float norm(float x, float y);
  float dot_product(float x1, float y1, float x2, float y2);

  std::vector<std::vector<cv::Point>> detect_straight_segments(
      const std::vector<cv::Point>& contour);

  void generate_gcode_optimized(const std::string& filename,
                                const cv::Mat& source, float scale,
                                float zHeight, float feedRate);

  void detect_lines_standard(const cv::Mat& edges, cv::Mat& output);
  void detect_lines_probabilistic(const cv::Mat& edges, cv::Mat& output);
};

#endif  // EDGE_DETECTION_HH
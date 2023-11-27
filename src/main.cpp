#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

#include "edge-detection.hh"
#include "file-util.hh"
#include "g-code.hh"

int main() {
  const std::string abs_image_path = FileUtil::getSingleFileAbsPath();
  if (!abs_image_path.empty()) {
    /* const cv::Mat src = cv::imread(abs_image_path);
    cv::imshow("image", src);
    cv::waitKey(); */
    cv::Mat_<uint8_t> img = cv::imread(abs_image_path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      std::cerr << "Failed to open the image. Exiting" << std::endl;
      return 1;
    }

    // Display the original image
    cv::imshow("Original", img);

    // Calculate the boundary

    EdgeDetection edge_detection;
    GCode gcode;

    std::vector<std::pair<int, int>> boundary =
        edge_detection.calculate_perimeter(img);
    cv::Mat canny_boundary = edge_detection.canny(img);
    cv::Mat_<uint8_t> canny_edge = edge_detection.canny_edge_detection(img);

    edge_detection.contour_wrapper(img);

    // Generate the G-code
    gcode.generate_gcode(boundary, "basic.gcode");
    edge_detection.generate_gcode_holes("holes.gcode", img, 0.1, 1.0, 100.0);

    // generate_gcode(contour, boundaryhole_file);
    edge_detection.generate_canny_gcode(canny_edge, "canny.gcode");

    cv::waitKey(0);
    cv::destroyAllWindows();
  }

  return 0;
}
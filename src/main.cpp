#include <iostream>
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
  int choice;
  std::string abs_image_path;
  EdgeDetection edge_detection;
  GCode gcode;

  while (true) {
    std::cout << "\n1 - Load file\n";
    std::cout << "2 - Exit\n";
    std::cout << "Enter your choice: ";
    std::cin >> choice;

    switch (choice) {
      case 1: {
        abs_image_path = FileUtil::getSingleFileAbsPath();
        if (!abs_image_path.empty()) {
          std::filesystem::path p(abs_image_path);
          std::cout << "\n" << p.filename() << " loaded!\n";
          std::cout << "1 - Generate G-code\n";
          std::cout << "2 - Load another file\n";
          std::cout << "3 - Exit\n";
          std::cout << "Enter your choice: ";
          std::cin >> choice;

          switch (choice) {
            case 1: {
              cv::Mat_<uint8_t> img = cv::imread(abs_image_path, cv::IMREAD_GRAYSCALE);
              if (img.empty()) {
                std::cerr << "Failed to open the image. Exiting" << std::endl;
                return 1;
              }

              // Display the original image
              cv::imshow("Original", img);

              // Calculate the boundary
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
              break;
            }
            case 2:
              continue;
            case 3:
              return 0;
            default:
              std::cout << "Invalid choice. Please enter 1, 2 or 3.\n";
              break;
          }
        }
        break;
      }
      case 2:
        return 0;
      default:
        std::cout << "Invalid choice. Please enter 1 or 2.\n";
        break;
    }
  }

  return 0;
}
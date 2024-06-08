#include <filesystem>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

#include "cnc-controller.hh"
#include "edge-detection.hh"
#include "file-util.hh"
#include "g-code.hh"

// Function to display the main menu
void showMainMenu() {
  std::cout << "\n1 - Load file\n";
  std::cout << "2 - CNC Operations\n";
  std::cout << "3 - Exit\n";
  std::cout << "Enter your choice: ";
}

// Function to display the contour menu
void showContourMenu(const std::string& filename) {
  std::cout << "\n" << filename << " loaded!\n";
  std::cout << "1 - Generate G-code\n";
  std::cout << "2 - Load another file\n";
  std::cout << "3 - CNC Operations\n";
  std::cout << "4 - Exit\n";
  std::cout << "Enter your choice: ";
}

// Function to display the CNC operations menu
void showGCodeMenu() {
  std::cout << "\n1 - Select Serial Port\n";
  std::cout << "2 - Load G-Code File\n";
  std::cout << "3 - Start CNC\n";
  std::cout << "4 - Stop and Reset CNC\n";
  std::cout << "5 - Exit\n";
  std::cout << "Enter your choice: ";
}

void cncOptions(int choice) {}

int main() {
  int choice;
  std::string abs_image_path;
  cv::Mat_<uint8_t> img;
  EdgeDetection edge_detection;
  GCode gcode;
  CNCController cnc_controller;

  while (true) {
    showMainMenu();
    std::cin >> choice;

    switch (choice) {
      case 1: {
        abs_image_path = FileUtil::getSingleFileAbsPath();
        if (!abs_image_path.empty()) {
          img = cv::imread(abs_image_path, cv::IMREAD_GRAYSCALE);
          if (img.empty()) {
            std::cerr << "Failed to open the image. Exiting" << std::endl;
            return 1;
          }
          std::filesystem::path p(abs_image_path);
          showContourMenu(p.filename().string());
          std::cin >> choice;

          switch (choice) {
            case 1: {
              // Calculate the boundary
              std::vector<std::pair<int, int>> boundary =
                  edge_detection.calculate_perimeter(img);
              cv::Mat canny_boundary = edge_detection.canny(img);
              cv::Mat_<uint8_t> canny_edge =
                  edge_detection.canny_edge_detection(img);

              edge_detection.contour_wrapper(img);

              // Generate the G-code
              gcode.generate_gcode(boundary, "basic.gcode");
              edge_detection.generate_gcode_holes("holes.gcode", img, 0.1, 1.0,
                                                  100.0);
              edge_detection.generate_canny_gcode(canny_edge, "canny.gcode");

              cv::waitKey(0);
              cv::destroyAllWindows();
              break;
            }
            case 2:
              continue;
            case 3: {
                int cncChoice;
                do {
                    showGCodeMenu();
                    std::cin >> cncChoice;

                    switch (cncChoice) {
                        case 1: {
                            std::string portname;
                            std::cout << "Enter the serial port (e.g., COM1 or /dev/ttyUSB0): ";
                            std::cin >> portname;
                            cnc_controller.setPortName(portname);
                            break;
                        }
                        case 2: {
                            std::string gcodeFilePath;
                            std::cout << "Enter the path to the G-code file: ";
                            std::cin >> gcodeFilePath;
                            cnc_controller.loadGCode(gcodeFilePath);
                            break;
                        }
                        case 3: {
                            cnc_controller.stream();
                            break;
                        }
                        case 4: {
                            cnc_controller.returnHome();
                            break;
                        }
                        case 5:
                            std::cout << "Exiting CNC operations menu." << std::endl;
                            break;
                        default:
                            std::cout << "Invalid choice. Please enter a number between 1 and 5." << std::endl;
                    }
                } while (cncChoice != 5);
              break;
            }
            case 4:
              return 0;
            default:
              std::cout
                  << "Invalid choice. Please enter a number between 1 and 5.\n";
              break;
          }
        }
        break;
      }
      case 2: {
                        int cncChoice;
                do {
                    showGCodeMenu();
                    std::cin >> cncChoice;

                    switch (cncChoice) {
                        case 1: {
                            std::string portname;
                            std::cout << "Enter the serial port (e.g., COM1 or /dev/ttyUSB0): ";
                            std::cin >> portname;
                            cnc_controller.setPortName(portname);
                            break;
                        }
                        case 2: {
                            std::string gcodeFilePath;
                            std::cout << "Enter the path to the G-code file: ";
                            std::cin >> gcodeFilePath;
                            cnc_controller.loadGCode(gcodeFilePath);
                            break;
                        }
                        case 3: {
                            cnc_controller.stream();
                            break;
                        }
                        case 4: {
                            cnc_controller.returnHome();
                            break;
                        }
                        case 5:
                            std::cout << "Exiting CNC operations menu." << std::endl;
                            break;
                        default:
                            std::cout << "Invalid choice. Please enter a number between 1 and 5." << std::endl;
                    }
                } while (cncChoice != 5);
        break;
      }
      case 3:
        return 0;
      default:
        std::cout << "Invalid choice. Please enter a number between 1 and 3.\n";
        break;
    }
  }

  return 0;
}

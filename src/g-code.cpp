#include "g-code.hh"
#include "edge-detection.hh"

void GCode::generate_gcode(const std::vector<std::pair<int, int>>& boundary,
    std::string filename, float scale, float feedRate) {
    std::ofstream file(filename);

    // G-code header
    file << "G21 ; Set to millimeters\n";
    file << "G90 ; Set to absolute positioning\n";
    file << "G92 X0.00 Y0.00 Z0.00 ; Set current position to origin\n\n";

    // Set starting point and pen up
    const auto& start = boundary.front();
    file << "G1 X" << start.second * scale << " Y" << -start.first * scale
        << " F" << feedRate << "\n";
    file << "M300 S50.00 ; Pen up\n";

    // Pen down to start drawing
    file << "M300 S30.00 ; Pen down\n";

    // Generate G01 commands for boundary scanning
    for (size_t i = 1; i < boundary.size(); i++) {
        file << "G1 X" << boundary[i].second * scale << " Y" << -boundary[i].first * scale
            << " F" << feedRate << "\n";
    }

    // Pen up after finishing drawing
    file << "M300 S50.00 ; Pen up\n";

    // G-code footer
    file << "G1 X0 Y0 F3500.00 ; Go home\n";
    file << "M18 ; Drives off\n";

    file.close();

    std::cout << "G-code file generated: " << filename << std::endl;
}

void EdgeDetection::generate_gcode_optimized(const std::string& filename,
    const cv::Mat& source, float scale,
    float zHeight, float feedRate) {
    std::ofstream file(filename);

    file << "G21 ; Set to millimeters\n";
    file << "G90 ; Set to absolute positioning\n";
    file << "G92 X0.00 Y0.00 Z0.00 ; Set current position to origin\n\n";

    // Iterate through outer contour and holes
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour_points);  // Add outer contour
    contours.insert(contours.end(), holes.begin(), holes.end());  // Add holes

    // Iterate through all contours
    for (const auto& contour : contours) {
        if (contour.size() < 3)  // Ensure there are enough points
            continue;

        // Detect straight line segments in the contour
        auto segments = detect_straight_segments(contour);

        // Move to the starting point of the first segment
        const auto& start = segments.front().front();
        file << "G1 X" << start.x * scale << " Y" << -start.y * scale << " F"
            << feedRate << '\n';

        // Pen down after moving to the starting point of the contour
        file << "M300 S30.00 ; Pen down\n";

        for (const auto& segment : segments) {
            if (segment.size() < 2) continue;

            // Move to the end point of the segment
            const auto& end = segment.back();
            file << "G1 X" << end.x * scale << " Y" << -end.y * scale << " F"
                << feedRate << '\n';
        }

        // Lift the pen after tracing the last segment
        file << "M300 S50.00 ; Pen up\n";
    }

    file << "G1 X0 Y0 F3500.00 ; Go home\n";
    //file << "M18 ; Drives off\n";

    // Close the file
    file.close();

    std::cout << "Optimized G-code file generated: " << filename << std::endl;
}

void EdgeDetection::generate_canny_gcode(const std::string& filename,
    const cv::Mat_<uint8_t>& edgeMap,
    float scale, float zHeight,
    float feedRate) {
    std::ofstream file(filename);

    file << "G21 ; Set to millimeters\n";
    file << "G90 ; Set to absolute positioning\n";
    file << "G92 X0.00 Y0.00 Z0.00 ; Set current position to origin\n\n";

    // Get all contours including holes
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edgeMap, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Iterate through all contours and hierarchy
    for (size_t i = 0; i < contours.size(); i++) {
        if (contours[i].size() < 3) continue;

        const auto& start = contours[i].front();
        file << "G1 X" << start.x * scale << " Y" << -start.y * scale << " F" << feedRate << '\n';
        file << "M300 S30.00 ; Pen down\n";

        for (size_t j = 1; j < contours[i].size(); ++j) {
            const auto& point = contours[i][j];
            file << "G1 X" << point.x * scale << " Y" << -point.y * scale << " F" << feedRate << '\n';
        }

        file << "M300 S50.00 ; Pen up\n";

        // Check for any holes
        for (size_t k = i + 1; k < contours.size(); ++k) {
            if (hierarchy[k][3] == i) {
                // Move to the starting point of the hole
                const auto& hole_start = contours[k].front();
                file << "G1 X" << hole_start.x * scale << " Y" << -hole_start.y * scale << " F" << feedRate << '\n';
                file << "M300 S30.00 ; Pen down\n";

                // Trace the hole contour
                for (size_t l = 1; l < contours[k].size(); ++l) {
                    const auto& hole_point = contours[k][l];
                    file << "G1 X" << hole_point.x * scale << " Y" << -hole_point.y * scale << " F" << feedRate << '\n';
                }

                file << "M300 S50.00 ; Pen up\n";
            }
        }
    }

    file << "G1 X0 Y0 F3500.00 ; Go home\n";
    //file << "M18 ; Drives off\n";

    file.close();

    std::cout << "Canny-based G-code file generated: " << filename << std::endl;
}


void EdgeDetection::generate_gcode_holes(const std::string& filename,
    const cv::Mat& source, float scale,
    float zHeight, float feedRate) {
    std::ofstream file(filename);

    file << "G21 ; Set to millimeters\n";
    file << "G90 ; Set to absolute positioning\n";
    file << "G92 X0.00 Y0.00 Z0.00 ; Set current position to origin\n\n";

    // Iterate through outer contour and holes
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour_points);  // Add outer contour
    contours.insert(contours.end(), holes.begin(), holes.end());  // Add holes

    // Iterate through all contours
    for (const auto& contour : contours) {
        if (contour.size() < 3)  // Ensure there are enough points
            continue;

        // Move to the starting point of the contour
        const auto& start = contour.front();
        file << "G1 X" << start.x * scale << " Y" << -start.y * scale << " F"
            << feedRate << '\n';

        // Pen down command after the first command of each border segment
        file << "M300 S30.00 ; Pen down\n";

        // Trace the contour (excluding the first point)
        for (size_t i = 1; i < contour.size(); ++i) {
            const auto& point = contour[i];
            file << "G1 X" << point.x * scale << " Y" << -point.y * scale << " F"
                << feedRate << '\n';
        }

        // Lift the pen after tracing the contour
        file << "M300 S50.00 ; Pen up\n";
    }

    // Write final G-code commands
    file << "G1 X0 Y0 F3500.00 ; Go home\n";
    file << "M18 ; Drives off\n";

    // Close the file
    file.close();

    std::cout << "G-code file generated: " << filename << std::endl;
}
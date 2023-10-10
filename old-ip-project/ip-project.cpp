#include "stdafx.h"
#include "common.h"
#include <fstream>

using namespace std;
using namespace cv;

bool is_inside(Mat img, int i, int j) {
    return (i >= 0 && i < img.rows && j >= 0 && j < img.cols);
}

/*----------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
EDGE DETECTION ON SIMPLE IMAGES (WITHOUT HOLE DETECTION)
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------*/
vector<pair<int, int>> calculate_perimeter(Mat_<uchar> img) {
    vector<pair<int, int>> boundary;
    int i, j;

    // Find P_0
    pair<int, int> P_0;
    for (i = 0; i < img.rows; i++) {
        for (j = 0; j < img.cols; j++) {
            if (img(i, j) == 0) {
                P_0 = make_pair(i, j);
                break;
            }
        }
        if (P_0.first != 0 || P_0.second != 0) {
            break;
        }
    }

    boundary.push_back(P_0);

    int n8_di[8] = { 0, -1, -1,  -1,  0,  1, 1, 1 };
    int n8_dj[8] = { 1,  1,  0,  -1, -1, -1, 0, 1 };

    int dir = 7;
    Point P_current(P_0.second, P_0.first);
    while (true) {
        int new_dir;
        if (dir % 2 == 0) {
            new_dir = (dir + 7) % 8;
        }
        else {
            new_dir = (dir + 6) % 8;
        }
        for (int k = 0; k < 8; k++) {
            int new_i = P_current.y + n8_di[new_dir];
            int new_j = P_current.x + n8_dj[new_dir];
            if (img(new_i, new_j) == 0) {
                P_current.x = new_j;
                P_current.y = new_i;
                boundary.push_back(make_pair(new_i, new_j));
                dir = new_dir;
                break;
            }
            new_dir = (new_dir + 1) % 8;
        }
        if (boundary.size() > 2 && boundary[0] == boundary[boundary.size() - 2] && boundary[boundary.size() - 1] == boundary[1]) {
            break;
        }
    }

    // Draw boundary on the image
    Mat boundary_img(img.rows, img.cols, CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < boundary.size() - 1; i++) {
        line(boundary_img, Point(boundary[i].second, boundary[i].first), Point(boundary[i + 1].second, boundary[i + 1].first), Scalar(0, 0, 255), 1);
    }

    imshow("Boundary", boundary_img);
    waitKey(0);

    return boundary;
}

void generate_gcode(vector<pair<int, int>> boundary, string filename) {
    ofstream file(filename);

    // G-code header
    file << "G90 ; Set to absolute positioning mode\n";
    file << "G21 ; Set units to millimeters\n";
    file << "G28 ; Home all axes\n";
    file << "\n";

    // Set starting point as G00 command
    file << "G00 X" << boundary[0].second << " Y" << boundary[0].first << endl;

    // Generate G01 commands for boundary scanning
    for (size_t i = 1; i < boundary.size(); i++) {
        file << "G01 X" << boundary[i].second << " Y" << boundary[i].first << endl;
    }

    // G-code footer
    file << "\n";
    file << "M2 ; End of program\n";

    cout << "G-code file generated: " << filename << endl;

    file.close();
}

/*----------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
EDGE DETECTION ON SIMPLE IMAGES (WITH HOLE DETECTION)
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------*/
// Next direction to follow while tracing the contour
const int yChainNeighbours[] = { 0, -1, -1, -1, 0, 1, 1, 1 };
const int xChainNeighbours[] = { 1, 1, 0, -1, -1, -1, 0, 1 };

// Neighboring pixels for labeling connected holes
const int yNeighbours[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
const int xNeighbours[] = { -1, 0, 1, -1, 1, -1, 0, 1 };

int totalHoles = 0;
Mat holeLabels;
Mat contour;
vector<Point> contourPoints;
vector<vector<Point>> holes;

// Function to fill a connected hole with a unique label
void fillLabel(Mat const& source, int x, int y)
{
    int nx, ny;
    Point aux;
    queue<Point> q;

    totalHoles++;
    holeLabels.at<uchar>(y, x) = totalHoles;
    q.push(Point(x, y));

    while (!q.empty())
    {
        aux = q.front();
        q.pop();

        for (int i = 0; i < 8; i++)
        {
            nx = aux.x + xNeighbours[i];
            ny = aux.y + yNeighbours[i];

            // Check if the pixel is inside the image and is not labeled
            if (is_inside(holeLabels, ny, nx) &&
                source.at<uchar>(ny, nx) == 255 &&
                !holeLabels.at<uchar>(ny, nx))
            {
                holeLabels.at<uchar>(ny, nx) = totalHoles;
                q.push(Point(nx, ny));
            }
        }
    }
}

// Function to trace the points of an inner hole using the chain code
vector<Point> traceHole(const Mat& source, int x, int y)
{
    vector<Point> holePoints;
    int dir = 7;
    Point aux, bux;

    // Function to check if the hole contour tracing is complete
    auto checkContourEnd = [](const vector<Point>& v) {
        int s = v.size();
        return s > 2 && v.front() == v.at(s - 2) && v.at(1) == v.back();
    };

    // Function to get the next direction for chain code tracing
    auto getNextDirection = [](const Mat& src, int r, int c, int d) {
        int k;
        if (d % 2 == 1)
            k = (d + 6) % 8;
        else
            k = (d + 7) % 8;

        for (int i = 8; i > 0; i--)
        {
            // Check if the neighbor pixel is inside the image and is part of the hole
            if (is_inside(src, r + yChainNeighbours[k], c + xChainNeighbours[k]) &&
                src.at<uchar>(r + yChainNeighbours[k], c + xChainNeighbours[k]) == 255)
                return k;

            k++;
            k %= 8;
        }

        return 0;
    };

    holePoints.push_back(Point(x, y));
    dir = getNextDirection(source, y, x, dir);
    aux = Point(x + xChainNeighbours[dir], y + yChainNeighbours[dir]);
    holePoints.push_back(aux);
    dir = getNextDirection(source, aux.y, aux.x, dir);

    while (!checkContourEnd(holePoints))
    {
        aux = holePoints.back();
        bux = Point(aux.x + xChainNeighbours[dir], aux.y + yChainNeighbours[dir]);
        holePoints.push_back(bux);
        dir = getNextDirection(source, bux.y, bux.x, dir);
    }

    return holePoints;
}

// Function to trace the outer contour of the image
void traceContour(const Mat source, int x, int y)
{
    int dir = 7;
    Point aux, bux;

    // Function to check if the contour tracing is complete
    auto checkContourEnd = [](const vector<Point>& v) {
        int s = v.size();
        return s > 2 && v.front() == v.at(s - 2) && v.at(1) == v.back();
    };

    // Function to get the next direction for chain code tracing
    auto getNextDirection = [](const Mat& src, int r, int c, int d) {
        int k;
        if (d % 2 == 1)
            k = (d + 6) % 8;
        else
            k = (d + 7) % 8;

        for (int i = 8; i > 0; i--)
        {
            // Check if the neighbor pixel is not part of the contour
            if (src.at<uchar>(r + yChainNeighbours[k], c + xChainNeighbours[k]) != 255)
                return k;

            k++;
            k %= 8;
        }

        return 0;
    };

    contourPoints.push_back(Point(x, y));
    dir = getNextDirection(source, y, x, dir);
    aux = Point(x + xChainNeighbours[dir], y + yChainNeighbours[dir]);
    contourPoints.push_back(aux);
    dir = getNextDirection(source, aux.y, aux.x, dir);

    while (!checkContourEnd(contourPoints))
    {
        aux = contourPoints.back();
        bux = Point(aux.x + xChainNeighbours[dir], aux.y + yChainNeighbours[dir]);
        contourPoints.push_back(bux);
        dir = getNextDirection(source, bux.y, bux.x, dir);
    }
}

// Function to draw the contour using the traced points
void drawContour(const Mat& source)
{
    contour = Mat(source.rows, source.cols, CV_8UC1, Scalar(255));

    // Draw the outer contour
    for (const auto p : contourPoints)
        contour.at<uchar>(p.y, p.x) = 0;

    // Draw the inner hole contours
    for (const auto h : holes)
        for (const auto p : h)
            contour.at<uchar>(p.y, p.x) = 0;

    imshow("Contour", contour);
}

// Main wrapper function to find and draw the contour
void contour_wrapper(Mat source)
{
    holeLabels = Mat::zeros(source.rows, source.cols, CV_8UC1);

    for (int i = 0; i < source.rows; ++i)
    {
        for (int j = 0; j < source.cols; ++j)
        {
            // If the pixel is not white and the outer contour points are not traced yet
            if (source.at<uchar>(i, j) != 255 && !contourPoints.size())
            {
                traceContour(source, j, i);
            }
            // If the pixel is white and the hole is not labeled
            else if (source.at<uchar>(i, j) == 255 && !holeLabels.at<uchar>(i, j))
            {
                fillLabel(source, j, i);
                holes.push_back(traceHole(source, j, i));
            }
        }
    }

    drawContour(source);
}

void generate_gcode_holes(const string& filename, const Mat& source, float scale, float zHeight, float feedRate)
{
    ofstream file(filename);

    file << "G90 ; Set to absolute positioning\n";
    file << "G21 ; Set to millimeters\n";
    file << "G92 X0 Y0 Z0 ; Set current position to origin\n";
    file << "G1 F" << feedRate << " ; Set feed rate\n";
    file << "G1 Z" << zHeight << " ; Move to initial Z height\n";

    // Trace the outer contour
    const auto& outerContour = contourPoints;
    if (outerContour.size() >= 3) // Ensure there are enough points
    {
        // Move to the starting point of the outer contour
        file << "G00 X" << outerContour[1].x * scale << " Y" << outerContour[1].y * scale << '\n';

        // Trace the outer contour (excluding the first and last points)
        for (size_t i = 2; i < outerContour.size() - 1; ++i)
        {
            const auto& point = outerContour[i];
            file << "G01 X" << point.x * scale << " Y" << point.y * scale << '\n';
        }
    }

    // Trace the inner holes
    for (const auto& hole : holes)
    {
        if (hole.size() >= 3) // Ensure there are enough points
        {
            // Move to the starting point of the hole contour
            file << "G00 X" << hole[1].x * scale << " Y" << hole[1].y * scale << '\n';

            // Trace the hole contour (excluding the first and last points)
            for (size_t i = 2; i < hole.size() - 1; ++i)
            {
                const auto& point = hole[i];
                file << "G01 X" << point.x * scale << " Y" << point.y * scale << '\n';
            }
        }
    }

    // Write final G-code commands
    file << "G1 Z" << zHeight << " ; Move to safe Z height\n";
    file << "M2 ; End of program\n";

    // Close the file
    file.close();

    cout << "G-code file generated: " << filename << endl;
}

/*----------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
CANNY EDGE DETECTION WITH BUILT-IN FUNCTION
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------*/
Mat canny(Mat img) {
    Mat img_gray;
    cvtColor(img, img_gray, COLOR_GRAY2BGR);

    Mat img_blur;
    GaussianBlur(img_gray, img_blur, Size(3, 3), 0);

    Mat edges;
    Canny(img_blur, edges, 100, 200, 3, false);

    imshow("Built-in Canny", edges);

    return edges;
}

/*----------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------
CANNY EDGE DETECTION
------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------*/
Mat_<float> convolution(Mat_<uchar> img, Mat_<float> H) {

    Mat_<float> conv = Mat_<float>::zeros(img.size());

    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            float sum = 0;
            for (int u = 0; u < H.rows; u++) {
                for (int v = 0; v < H.cols; v++) {
                    if (is_inside(img, i + u - H.rows / 2, j + v - H.cols / 2))
                        sum += H(u, v) * img(i + u - H.rows / 2, j + v - H.cols / 2);
                }
            }
            conv(i, j) = sum;
        }
    }
    return conv;
}

Mat_<uchar> canny_edge_detection(Mat_<uchar> img) {
    // 1. Gradient
    Mat_<float> sobelX = (Mat_<float>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
    Mat_<float> sobelY = (Mat_<float>(3, 3) << 1, 2, 1, 0, 0, 0, -1, -2, -1);

    Mat_<float> dx = convolution(img, sobelX);
    Mat_<float> dy = convolution(img, sobelY);

    // imshow("dx", abs(dx)/255);
    // imshow("dy", abs(dy)/255);

    // 2. Magnitude and angle
    Mat_<float> mag(img.rows, img.cols);
    Mat_<float> ang(img.rows, img.cols);

    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            mag(i, j) = sqrt(dx(i, j) * dx(i, j) + dy(i, j) * dy(i, j));
            ang(i, j) = atan2(dy(i, j), dx(i, j));
        }
    }

    Mat_<float> magCopy(img.rows, img.cols);
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            magCopy(i, j) = mag(i, j) / (4 * sqrt(2) * 255);
        }
    }

    //imshow("mag", magCopy);

    // 3. Quantize the angles
    Mat_<int> q(img.rows, img.cols);
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            float a = ang(i, j);
            if (a < 0) {
                a += 2 * PI;
            }
            q(i, j) = int(round(a / (2 * PI) * 8)) % 8;
        }
    }

    // 4. Edge thinning
    int di[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
    int dj[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

    Mat_<float> magCopy2 = mag.clone();  // Create a copy of mag matrix

    for (int i = 1; i < img.rows - 1; i++) {
        for (int j = 1; j < img.cols - 1; j++) {
            int qi = q(i, j);
            float m = mag(i, j);

            // Non-maximum suppression
            float m1, m2;
            if (qi == 0 || qi == 4) {
                m1 = mag(i, j - 1);
                m2 = mag(i, j + 1);
            }
            else if (qi == 1 || qi == 5) {
                m1 = mag(i - 1, j + 1);
                m2 = mag(i + 1, j - 1);
            }
            else if (qi == 2 || qi == 6) {
                m1 = mag(i - 1, j);
                m2 = mag(i + 1, j);
            }
            else {
                m1 = mag(i - 1, j - 1);
                m2 = mag(i + 1, j + 1);
            }

            if (m < m1 || m < m2) {
                magCopy2(i, j) = 0;
            }
        }
    }

    //imshow("edge thinning", magCopy2);

    // 5. Thresholding
    float lowThreshold = 0.1;
    float highThreshold = 0.3;

    Mat_<uchar> edgeMap = Mat_<uchar>(img.rows, img.cols, (uchar)0);

    for (int i = 1; i < img.rows - 1; i++) {
        for (int j = 1; j < img.cols - 1; j++) {
            float t1 = highThreshold * 255;
            float t2 = lowThreshold * 255;

            if (magCopy2(i, j) >= t1) {
                edgeMap(i, j) = 255;
            }
            else if (magCopy2(i, j) >= t2 && (magCopy2(i - 1, j) >= t1 || magCopy2(i + 1, j) >= t1 ||
                magCopy2(i, j - 1) >= t1 || magCopy2(i, j + 1) >= t1 ||
                magCopy2(i - 1, j - 1) >= t1 || magCopy2(i - 1, j + 1) >= t1 ||
                magCopy2(i + 1, j - 1) >= t1 || magCopy2(i + 1, j + 1) >= t1)) {
                edgeMap(i, j) = 255;
            }
        }
    }

    imshow("Explicit Canny", edgeMap);
    waitKey(0);

    return edgeMap;
}

void generate_canny_gcode(Mat_<uchar> edges, string filename) {
    // Find contours
    vector<vector<Point>> contours;
    findContours(edges.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Find the largest contour (boundary of the image)
    int maxArea = 0;
    size_t maxIdx = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        int area = contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            maxIdx = i;
        }
    }

    // Extract the boundary points
    vector<Point>& boundary = contours[maxIdx];

    // Generate G-code file
    ofstream file(filename);

    // G-code header
    file << "G90 ; Set to absolute positioning mode\n";
    file << "G21 ; Set units to millimeters\n";
    file << "G28 ; Home all axes\n";
    file << "\n";

    // Set starting point as G00 command
    file << "G00 X" << boundary[0].x << " Y" << boundary[0].y << endl;

    // Generate G01 commands for boundary scanning
    for (size_t i = 1; i < boundary.size(); i++) {
        file << "G01 X" << boundary[i].x << " Y" << boundary[i].y << endl;
    }

    // G-code footer
    file << "\n";
    file << "M2 ; End of program\n";

    file.close();
}



int main() {
    string imagePath;

menu:
    // Prompt the user to select an image
    int option;
    cout << "Select an image:\n";
    cout << "1. A.bmp\n";
    cout << "2. B.bmp\n";
    cout << "3. C.bmp\n";
    cout << "4. F.bmp\n";
    cout << "ESC: Exit\n";
    cout << "Enter your choice: ";
    cin >> option;

    // Set the image path based on the selected option
    switch (option) {
        case 1:
            imagePath = "..\\PROJECT\\LETTER IMAGES\\A.bmp";
            break;
        case 2:
            imagePath = "..PROJECT\\LETTER IMAGES\\B.bmp";
            break;
        case 3:
            imagePath = "..\\PROJECT\\LETTER IMAGES\\C.bmp";
            break;
        case 4:
            imagePath = "..\\PROJECT\\LETTER IMAGES\\F.bmp";
            break;
        default:
            std::cerr << "Invalid option. Exiting.\n";
            return 1;
    }

    // Read the selected image
    Mat img = imread(imagePath, IMREAD_GRAYSCALE);
    if (img.empty()) {
        cerr << "Failed to open the image. Exiting.\n";
        return 1;
    }

    // Display the original image
    imshow("Original", img);

    // Calculate the boundary
    vector<pair<int, int>> boundary = calculate_perimeter(img);
    Mat canny_boundary = canny(img);
    Mat_<uchar> canny_edge = canny_edge_detection(img);

    contour_wrapper(img);

    // Generate the G-code
    generate_gcode(boundary, "basic.gcode");
    generate_gcode_holes("holes.gcode", img, 0.1, 1.0, 100.0);

    //generate_gcode(contour, boundaryhole_file);
    generate_canny_gcode(canny_edge, "canny.gcode");

    waitKey(0);
    destroyAllWindows();

    goto menu;

    return 0;

}

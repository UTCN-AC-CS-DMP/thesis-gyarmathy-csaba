#include "edge-detection.hh"

std::vector<std::pair<int, int>> EdgeDetection::calculate_perimeter(
    const cv::Mat_<uint8_t>& img) {
  std::vector<std::pair<int, int>> boundary;
  int i, j;

  // Find P_0
  std::pair<int, int> P_0;
  for (i = 0; i < img.rows; i++) {
    for (j = 0; j < img.cols; j++) {
      if (img(i, j) == 0) {
        P_0 = std::make_pair(i, j);
        break;
      }
    }
    if (P_0.first != 0 || P_0.second != 0) {
      break;
    }
  }

  boundary.push_back(P_0);

  int n8_di[8] = {0, -1, -1, -1, 0, 1, 1, 1};
  int n8_dj[8] = {1, 1, 0, -1, -1, -1, 0, 1};

  int dir = 7;
  cv::Point P_current(P_0.second, P_0.first);
  while (true) {
    int new_dir;
    if (dir % 2 == 0) {
      new_dir = (dir + 7) % 8;
    } else {
      new_dir = (dir + 6) % 8;
    }
    for (int k = 0; k < 8; k++) {
      int new_i = P_current.y + n8_di[new_dir];
      int new_j = P_current.x + n8_dj[new_dir];
      if (img(new_i, new_j) == 0) {
        P_current.x = new_j;
        P_current.y = new_i;
        boundary.push_back(std::make_pair(new_i, new_j));
        dir = new_dir;
        break;
      }
      new_dir = (new_dir + 1) % 8;
    }
    if (boundary.size() > 2 && boundary[0] == boundary[boundary.size() - 2] &&
        boundary[boundary.size() - 1] == boundary[1]) {
      break;
    }
  }

  // Draw boundary on the image
  cv::Mat boundary_img(img.rows, img.cols, CV_8UC3, cv::Scalar(255, 255, 255));
  for (int i = 0; i < boundary.size() - 1; i++) {
    cv::line(boundary_img, cv::Point(boundary[i].second, boundary[i].first),
             cv::Point(boundary[i + 1].second, boundary[i + 1].first),
             cv::Scalar(0, 0, 255), 1);
  }

  cv::imshow("Boundary", boundary_img);
  cv::waitKey(0);

  return boundary;
}

void EdgeDetection::fill_label(const cv::Mat& source, int x, int y) {
  int nx, ny;
  cv::Point aux;
  std::queue<cv::Point> q;

  total_holes++;
  hole_labels.at<uint8_t>(y, x) = total_holes;
  q.push(cv::Point(x, y));

  while (!q.empty()) {
    aux = q.front();
    q.pop();

    for (int i = 0; i < 8; i++) {
      nx = aux.x + x_neighbours[i];
      ny = aux.y + y_neighbours[i];

      // Check if the pixel is inside the image and is not labeled
      if (ImageUtil::is_inside(hole_labels, ny, nx) &&
          source.at<uint8_t>(ny, nx) == 255 &&
          !hole_labels.at<uint8_t>(ny, nx)) {
        hole_labels.at<uint8_t>(ny, nx) = total_holes;
        q.push(cv::Point(nx, ny));
      }
    }
  }
}

std::vector<cv::Point> EdgeDetection::trace_hole(const cv::Mat& source, int x,
                                                 int y) {
  std::vector<cv::Point> holePoints;
  int dir = 7;
  cv::Point aux, bux;

  // Function to check if the hole contour tracing is complete
  auto checkContourEnd = [](const std::vector<cv::Point>& v) {
    int s = v.size();
    return s > 2 && v.front() == v.at(s - 2) && v.at(1) == v.back();
  };

  // Function to get the next direction for chain code tracing
  auto getNextDirection = [this](const cv::Mat& src, int r, int c, int d) {
    int k;
    if (d % 2 == 1)
      k = (d + 6) % 8;
    else
      k = (d + 7) % 8;

    for (int i = 8; i > 0; i--) {
      // Check if the neighbor pixel is inside the image and is part of the hole
      if (ImageUtil::is_inside(src, r + y_chain_neighbours[k],
                               c + x_chain_neighbours[k]) &&
          src.at<uint8_t>(r + y_chain_neighbours[k],
                          c + x_chain_neighbours[k]) == 255)
        return k;

      k++;
      k %= 8;
    }

    return 0;
  };

  holePoints.push_back(cv::Point(x, y));
  dir = getNextDirection(source, y, x, dir);
  aux = cv::Point(x + x_chain_neighbours[dir], y + y_chain_neighbours[dir]);
  holePoints.push_back(aux);
  dir = getNextDirection(source, aux.y, aux.x, dir);

  while (!checkContourEnd(holePoints)) {
    aux = holePoints.back();
    bux = cv::Point(aux.x + x_chain_neighbours[dir],
                    aux.y + y_chain_neighbours[dir]);
    holePoints.push_back(bux);
    dir = getNextDirection(source, bux.y, bux.x, dir);
  }

  return holePoints;
}

void EdgeDetection::trace_contour(const cv::Mat& source, int x, int y) {
  int dir = 7;
  cv::Point aux, bux;

  // Function to check if the contour tracing is complete
  auto checkContourEnd = [](const std::vector<cv::Point>& v) {
    int s = v.size();
    return s > 2 && v.front() == v.at(s - 2) && v.at(1) == v.back();
  };

  // Function to get the next direction for chain code tracing
  auto getNextDirection = [this](const cv::Mat& src, int r, int c, int d) {
    int k;
    if (d % 2 == 1)
      k = (d + 6) % 8;
    else
      k = (d + 7) % 8;

    for (int i = 8; i > 0; i--) {
      // Check if the neighbor pixel is not part of the contour
      if (src.at<uint8_t>(r + y_chain_neighbours[k],
                          c + x_chain_neighbours[k]) != 255)
        return k;

      k++;
      k %= 8;
    }

    return 0;
  };

  contour_points.push_back(cv::Point(x, y));
  dir = getNextDirection(source, y, x, dir);
  aux = cv::Point(x + x_chain_neighbours[dir], y + y_chain_neighbours[dir]);
  contour_points.push_back(aux);
  dir = getNextDirection(source, aux.y, aux.x, dir);

  while (!checkContourEnd(contour_points)) {
    aux = contour_points.back();
    bux = cv::Point(aux.x + x_chain_neighbours[dir],
                    aux.y + y_chain_neighbours[dir]);
    contour_points.push_back(bux);
    dir = getNextDirection(source, bux.y, bux.x, dir);
  }
}

void EdgeDetection::draw_contour(const cv::Mat& source) {
  contour = cv::Mat(source.rows, source.cols, CV_8UC1, cv::Scalar(255));

  // Draw the outer contour
  for (const auto p : contour_points) contour.at<uint8_t>(p.y, p.x) = 0;

  // Draw the inner hole contours
  for (const auto& h : holes)
    for (const auto p : h) contour.at<uint8_t>(p.y, p.x) = 0;

  imshow("Contour", contour);
}

void EdgeDetection::contour_wrapper(cv::Mat& source) {
    // Ensure the image is grayscale
    cv::Mat gray;
    if (source.channels() > 1) {
        cv::cvtColor(source, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        gray = source;
    }

    // Apply Canny edge detection
    cv::Mat edges = canny(gray);

    // Initialize output images for line detection
    cv::Mat dst, cdst, cdstP;
    cv::cvtColor(edges, dst, cv::COLOR_GRAY2BGR);
    cdst = dst.clone();
    cdstP = dst.clone();

    // Detect and draw lines using standard and probabilistic Hough transforms
    detect_lines_standard(edges, cdst);
    detect_lines_probabilistic(edges, cdstP);
    cv::imshow("Standard Hough", cdst);
    cv::imshow("Probabilistic Hough", cdstP);

    // Clear previous contour data
    contour_points.clear();
    holes.clear();
    hole_labels = cv::Mat::zeros(source.rows, source.cols, CV_8UC1);

    for (int i = 0; i < source.rows; ++i) {
        for (int j = 0; j < source.cols; ++j) {
            if (source.at<uint8_t>(i, j) != 255 && contour_points.empty()) {
                trace_contour(source, j, i);
            }
            else if (source.at<uint8_t>(i, j) == 255 &&
                !hole_labels.at<uint8_t>(i, j)) {
                fill_label(source, j, i);
                holes.push_back(trace_hole(source, j, i));
            }
        }
    }

    draw_contour(source);
    cv::waitKey(0);
}

cv::Mat EdgeDetection::canny(const cv::Mat& img) {
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, cv::COLOR_GRAY2BGR);

  cv::Mat img_blur;
  cv::GaussianBlur(img_gray, img_blur, cv::Size(3, 3), 0);

  cv::Mat edges;
  cv::Canny(img_blur, edges, 100, 200, 3, false);

  cv::imshow("Built-in Canny", edges);

  return edges;
}

cv::Mat_<float> EdgeDetection::convolution(cv::Mat_<uint8_t>& img,
                                           cv::Mat_<float>& H) {
  cv::Mat_<float> conv = cv::Mat_<float>::zeros(img.size());

  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      float sum = 0;
      for (int u = 0; u < H.rows; u++) {
        for (int v = 0; v < H.cols; v++) {
          if (ImageUtil::is_inside(img, i + u - H.rows / 2, j + v - H.cols / 2))
            sum += H(u, v) * img(i + u - H.rows / 2, j + v - H.cols / 2);
        }
      }
      conv(i, j) = sum;
    }
  }
  return conv;
}

cv::Mat_<uint8_t> EdgeDetection::canny_edge_detection(cv::Mat_<uint8_t>& img) {
  // 1. Gradient
  cv::Mat_<float> sobelX =
      (cv::Mat_<float>(3, 3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
  cv::Mat_<float> sobelY =
      (cv::Mat_<float>(3, 3) << 1, 2, 1, 0, 0, 0, -1, -2, -1);

  cv::Mat_<float> dx = convolution(img, sobelX);
  cv::Mat_<float> dy = convolution(img, sobelY);

  // imshow("dx", abs(dx)/255);
  // imshow("dy", abs(dy)/255);

  // 2. Magnitude and angle
  cv::Mat_<float> mag(img.rows, img.cols);
  cv::Mat_<float> ang(img.rows, img.cols);

  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      mag(i, j) = sqrt(dx(i, j) * dx(i, j) + dy(i, j) * dy(i, j));
      ang(i, j) = atan2(dy(i, j), dx(i, j));
    }
  }

  cv::Mat_<float> magCopy(img.rows, img.cols);
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      magCopy(i, j) = mag(i, j) / (4 * sqrt(2) * 255);
    }
  }

  // imshow("mag", magCopy);

  // 3. Quantize the angles
  cv::Mat_<int> q(img.rows, img.cols);
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      float a = ang(i, j);
      if (a < 0) {
        a += 2 * PI_20_PREC;
      }
      q(i, j) = int(round(a / (2 * PI_20_PREC) * 8)) % 8;
    }
  }

  // 4. Edge thinning
  int di[] = {-1, -1, 0, 1, 1, 1, 0, -1};
  int dj[] = {0, 1, 1, 1, 0, -1, -1, -1};

  cv::Mat_<float> magCopy2 = mag.clone();  // Create a copy of mag matrix

  for (int i = 1; i < img.rows - 1; i++) {
    for (int j = 1; j < img.cols - 1; j++) {
      int qi = q(i, j);
      float m = mag(i, j);

      // Non-maximum suppression
      float m1, m2;
      if (qi == 0 || qi == 4) {
        m1 = mag(i, j - 1);
        m2 = mag(i, j + 1);
      } else if (qi == 1 || qi == 5) {
        m1 = mag(i - 1, j + 1);
        m2 = mag(i + 1, j - 1);
      } else if (qi == 2 || qi == 6) {
        m1 = mag(i - 1, j);
        m2 = mag(i + 1, j);
      } else {
        m1 = mag(i - 1, j - 1);
        m2 = mag(i + 1, j + 1);
      }

      if (m < m1 || m < m2) {
        magCopy2(i, j) = 0;
      }
    }
  }

  // imshow("edge thinning", magCopy2);

  // 5. Thresholding
  float lowThreshold = 0.1;
  float highThreshold = 0.3;

  cv::Mat_<uint8_t> edgeMap = cv::Mat_<uint8_t>(img.rows, img.cols, (uint8_t)0);

  for (int i = 1; i < img.rows - 1; i++) {
    for (int j = 1; j < img.cols - 1; j++) {
      float t1 = highThreshold * 255;
      float t2 = lowThreshold * 255;

      if (magCopy2(i, j) >= t1) {
        edgeMap(i, j) = 255;
      } else if (magCopy2(i, j) >= t2 &&
                 (magCopy2(i - 1, j) >= t1 || magCopy2(i + 1, j) >= t1 ||
                  magCopy2(i, j - 1) >= t1 || magCopy2(i, j + 1) >= t1 ||
                  magCopy2(i - 1, j - 1) >= t1 ||
                  magCopy2(i - 1, j + 1) >= t1 ||
                  magCopy2(i + 1, j - 1) >= t1 ||
                  magCopy2(i + 1, j + 1) >= t1)) {
        edgeMap(i, j) = 255;
      }
    }
  }

  cv::imshow("Explicit Canny", edgeMap);
  cv::waitKey(0);

  return edgeMap;
}

bool EdgeDetection::approx_equal(float a, float b) {
  return std::fabs(a - b) < epsilon;
}

float EdgeDetection::norm(float x, float y) {
    return std::sqrt(x * x + y * y);
}

float EdgeDetection::dot_product(float x1, float y1, float x2, float y2) {
    return x1 * x2 + y1 * y2;
}

std::vector<std::vector<cv::Point>> EdgeDetection::detect_straight_segments(const std::vector<cv::Point>& contour) {
    std::vector<std::vector<cv::Point>> segments;
    size_t n = contour.size();
    if (n < 2) return segments;

    std::vector<cv::Point> current_segment;
    current_segment.push_back(contour[0]);

    for (size_t i = 1; i < n; ++i) {
        cv::Point prev = contour[i - 1];
        cv::Point curr = contour[i];
        cv::Point next = contour[(i + 1) % n];

        current_segment.push_back(curr);

        float dx = static_cast<float>(curr.x - prev.x);
        float dy = static_cast<float>(curr.y - prev.y);

        float next_dx = static_cast<float>(next.x - curr.x);
        float next_dy = static_cast<float>(next.y - curr.y);

        float norm_current = norm(dx, dy);
        float norm_next = norm(next_dx, next_dy);

        if (norm_current > 0 && norm_next > 0) {
            float dot = dot_product(dx, dy, next_dx, next_dy);

            float cos_theta = dot / (norm_current * norm_next);

            if (!approx_equal(std::fabs(cos_theta), 1.0f)) {
                segments.push_back(current_segment);
                current_segment.clear();
                current_segment.push_back(curr);
            }
        }
    }

    if (!current_segment.empty()) {
        segments.push_back(current_segment);
    }

    return segments;
}

// Built-in OpenCV functions for line detection
void EdgeDetection::detect_lines_standard(const cv::Mat& edges,
                                          cv::Mat& output) {
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(edges, lines, 1, PI_20_PREC / 180, 150, 0, 0);

  for (size_t i = 0; i < lines.size(); i++) {
    float rho = lines[i][0], theta = lines[i][1];
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a * rho, y0 = b * rho;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));
    cv::line(output, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  }
}

void EdgeDetection::detect_lines_probabilistic(const cv::Mat& edges,
                                               cv::Mat& output) {
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edges, lines, 1, PI_20_PREC / 180, 50, 50, 10);

  for (size_t i = 0; i < lines.size(); i++) {
    cv::Vec4i l = lines[i];
    cv::line(output, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
             cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  }
}

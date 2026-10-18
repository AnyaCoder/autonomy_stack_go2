// Standalone regression test for the image routine used by ContourDetector.
#include "far_planner/contour_image.h"
#include <iostream>
#include <stdexcept>

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

cv::Mat oldImage(const cv::Mat& counts) {
    cv::Mat support, result;
    cv::boxFilter(counts, support, -1, cv::Size(3, 3), cv::Point(-1, -1),
                  false, cv::BORDER_CONSTANT);
    result = support > 4;
    cv::resize(result, result, cv::Size(), 3, 3, cv::INTER_LINEAR);
    cv::boxFilter(result, result, -1, cv::Size(4, 4), cv::Point(-1, -1), false);
    return result;
}

int corridorGap(const cv::Mat& image) {
    const int row = image.rows / 2, center = image.cols / 2;
    int left = center, right = center;
    while (left >= 0 && image.at<unsigned char>(row, left) == 0) --left;
    while (right < image.cols && image.at<unsigned char>(row, right) == 0) ++right;
    check(left >= 0 && right < image.cols, "missing corridor wall");
    return right - left - 1;
}

int main() {
    cv::Mat counts = cv::Mat::zeros(61, 61, CV_32FC1);
    // Walls 1 m apart, dense returns, 0.1 m source cells.
    for (int row = 5; row < 56; ++row) {
        counts.at<float>(row, 25) = 5;
        counts.at<float>(row, 35) = 5;
    }
    const auto narrow = far_contour::BuildImage(counts, 3, 4, true, .1, .02);
    const auto original = oldImage(counts);
    const int old_gap = corridorGap(original), new_gap = corridorGap(narrow);
    check(new_gap > old_gap, "contour change did not recover corridor width");
    check(new_gap == 25, "2 cm padding should expand by one resized pixel per side");
    // With no padding a supported cell stays exactly one source cell wide.
    const auto unpadded = far_contour::BuildImage(counts, 3, 4, true, .1, 0);
    check(corridorGap(unpadded) == 27, "neighbour support became occupied cells");

    cv::Mat sparse = cv::Mat::zeros(61, 61, CV_32FC1);
    // Sparse occupied cells, with multiple vertical returns at each wall cell.
    for (int row = 6; row < 55; row += 2) sparse.at<float>(row, 25) = 5;
    auto supported = far_contour::BuildImage(sparse, 3, 4, true, .1, .02);
    for (int row = 6; row < 55; row += 2)
        check(supported.at<unsigned char>(row * 3 + 1, 25 * 3 + 1) > 0,
              "lost supported sparse wall return");
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(supported.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    check(contours.size() == 1, "sparse wall split into disconnected islands");

    cv::Mat noise = cv::Mat::zeros(61, 61, CV_32FC1);
    noise.at<float>(30, 30) = 1;
    check(cv::countNonZero(far_contour::BuildImage(noise, 3, 4, true, .1, .02)) == 0,
          "isolated noise passed density filter");
    noise.at<float>(30, 30) = 5;
    check(cv::countNonZero(far_contour::BuildImage(noise, 3, 4, true, .1, .02)) > 0,
          "small obstacle with supported returns disappeared");
    check(cv::countNonZero(far_contour::BuildImage(
              cv::Mat::zeros(61, 61, CV_32FC1), 3, 4, true, .1, .02)) == 0,
          "empty scene gained obstacles");
    std::cout << "1 m synthetic corridor: image gap " << old_gap * .1 / 3
              << " -> " << new_gap * .1 / 3 << " m; sparse wall/noise/small obstacle checks passed\n";
}

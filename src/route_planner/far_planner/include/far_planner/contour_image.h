#ifndef FAR_PLANNER_CONTOUR_IMAGE_H
#define FAR_PLANNER_CONTOUR_IMAGE_H

#include <cmath>
#include <opencv2/opencv.hpp>

namespace far_contour {

// Count neighbouring returns to reject noise, but keep only cells with actual
// returns. Neighbour support must not itself become an occupied obstacle cell.
inline cv::Mat BuildImage(const cv::Mat& counts, double resize_ratio,
                          int support_threshold, bool filter_noise,
                          double voxel_dim, double padding_m) {
    cv::Mat occupied = counts > 0;
    if (filter_noise) {
        cv::Mat support;
        cv::boxFilter(counts, support, -1, cv::Size(3, 3),
                      cv::Point(-1, -1), false, cv::BORDER_CONSTANT);
        cv::bitwise_and(occupied, support > support_threshold, occupied);
    }
    cv::Mat image;
    cv::resize(occupied, image, cv::Size(), resize_ratio, resize_ratio,
               cv::INTER_NEAREST);
    const int radius = static_cast<int>(std::ceil(padding_m * resize_ratio / voxel_dim));
    if (radius > 0) {
        cv::dilate(image, image, cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(2 * radius + 1, 2 * radius + 1)));
        cv::morphologyEx(image, image, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    }
    return image;
}

}  // namespace far_contour
#endif

#include "Viewer.h"
#include <opencv2/highgui.hpp>

Viewer::Viewer(const std::string& window_name) : window_name_(window_name) {
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
}

Viewer::~Viewer() {
    cv::destroyWindow(window_name_);
}

void Viewer::show(const cv::Mat& frame) {
    cv::imshow(window_name_, frame);
}

int Viewer::waitKey(int delay) {
    return cv::waitKey(delay);
}

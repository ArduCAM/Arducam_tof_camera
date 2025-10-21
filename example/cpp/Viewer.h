#ifndef VIEWER_H
#define VIEWER_H

#include <opencv2/core.hpp>
#include <string>

class Viewer {
public:
    Viewer(const std::string& window_name);
    ~Viewer();

    void show(const cv::Mat& frame);
    int waitKey(int delay);

private:
    std::string window_name_;
};

#endif // VIEWER_H

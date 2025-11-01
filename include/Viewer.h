#ifndef VIEWER_H
#define VIEWER_H

#include <opencv2/core.hpp>
#include <string>

/**
 * @brief Handles the display of video frames using OpenCV.
 *
 * This class provides a simple interface for displaying images in a window and
 * waiting for user input.
 */
class Viewer {
public:
    /**
     * @brief Construct a new Viewer object.
     *
     * @param window_name The name of the window to be created.
     */
    Viewer(const std::string& window_name);
    /**
     * @brief Destroy the Viewer object.
     */
    ~Viewer();

    /**
     * @brief Displays a frame in the window.
     *
     * @param frame The frame to be displayed.
     */
    void show(const cv::Mat& frame);
    /**
     * @brief Waits for a key press for a specified amount of time.
     *
     * @param delay The time to wait in milliseconds.
     * @return int The code of the pressed key, or -1 if no key was pressed.
     */
    int waitKey(int delay);

private:
    std::string window_name_;
};

#endif // VIEWER_H

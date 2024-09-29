#include "ArducamTOFCamera.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace Arducam;

void display_fps(void)
{
    using std::chrono::high_resolution_clock;
    using namespace std::literals;
    static int count = 0;
    static auto time_beg = high_resolution_clock::now();
    auto time_end = high_resolution_clock::now();
    ++count;
    auto duration_ms = (time_end - time_beg) / 1ms;
    if (duration_ms >= 1000) {
        std::cout << "fps:" << count << std::endl;
        count = 0;
        time_beg = time_end;
    }
}

bool processKey()
{
    int key = cv::waitKey(1);
    switch (key) {
    case 27:
    case 'q':
        return false;
    default:
        break;
    }
    return true;
}

int main()
{
    ArducamTOFCamera tof;
    ArducamFrameBuffer* frame;
    if (tof.open(Connection::CSI)) {
        std::cerr << "initialization failed" << std::endl;
        return -1;
    }
    if (tof.start(FrameType::RAW_FRAME)) {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }
    //  Modify the range also to modify the MAX_DISTANCE
    int max_range = 4000;
    // tof.setControl(Control::RANGE, 4000);
    tof.getControl(Control::RANGE, &max_range);

    CameraInfo info = tof.getCameraInfo();
    std::cout << "open camera with (" << info.width << "x" << info.height << ") with range " << max_range << std::endl;

    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);

    for (;;) {
        frame = tof.requestFrame(2000);
        if (frame == nullptr) {
            continue;
        }
        FrameFormat format;
        frame->getFormat(FrameType::RAW_FRAME, format);
        std::cout << "frame: (" << format.width << "x" << format.height << ")" << std::endl;

        int16_t* raw_ptr = (int16_t*)frame->getData(FrameType::RAW_FRAME);
        if (raw_ptr == nullptr) {
            tof.releaseFrame(frame);
            continue;
        }

        cv::Mat result_frame(format.height, format.width, CV_8U);
        cv::Mat raw_frame(format.width, format.height, CV_16S, raw_ptr);

        raw_frame.convertTo(result_frame, CV_8U, 1. / (1 << 4), 0);
        cv::imshow("preview", result_frame);

        if (!processKey()) {
            break;
        }
        display_fps();
        tof.releaseFrame(frame);
    }

    if (tof.stop()) {
        return -1;
    }

    if (tof.close()) {
        return -1;
    }

    return 0;
}

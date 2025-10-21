#include "TOFApplication.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

TOFApplication::TOFApplication() {}

TOFApplication::~TOFApplication() {}

int TOFApplication::initialize() {
    camera_ = std::make_unique<Camera>(Arducam::Connection::USB);
    if (!camera_->open()) {
        return -1;
    }

    auto info = camera_->getCameraInfo();
    std::cout << "open camera with (" << info.width << "x" << info.height << ")" << std::endl;

    if (!camera_->start(Arducam::FrameType::DEPTH_FRAME)) {
        return -1;
    }

    camera_->setControl(Arducam::Control::RANGE, 4);
    viewer_ = std::make_unique<Viewer>("preview");

    return 0;
}

void TOFApplication::run() {
    for (;;) {
        Arducam::ArducamFrameBuffer* frame = camera_->requestFrame(200);
        if (frame == nullptr) {
            continue;
        }

        Arducam::FrameFormat format;
        frame->getFormat(Arducam::FrameType::DEPTH_FRAME, format);

        float* depth_ptr = (float*)frame->getData(Arducam::FrameType::DEPTH_FRAME);

        cv::Mat depth_frame(format.height, format.width, CV_32F, depth_ptr);
        cv::Mat result_frame;
        depth_frame.convertTo(result_frame, CV_8U, 255.0 / 7000, 0);

        cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_RAINBOW);

        viewer_->show(result_frame);

        auto key = viewer_->waitKey(1);
        if (key == 27 || key == 'q') {
            break;
        }

        camera_->releaseFrame(frame);
    }
}

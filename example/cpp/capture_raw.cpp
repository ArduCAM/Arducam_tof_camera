#include "ArduCamTOFCamera.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iostream>

#define MAX_DISTANCE 4

int main()
{
    ArduCam::ArduCamTOFCamera tof;
    ArduCam::FrameBuffer *frame;
    if (tof.init(ArduCam::CSI,ArduCam::RAW_TYPE)){
        std::cerr<<"initialization failed"<<std::endl;
        exit(-1);
    }
    if (tof.start()){
        std::cerr<<"Failed to start camera"<<std::endl;
        exit(-1);
    }
    //  Modify the range also to modify the MAX_DISTANCE
    tof.setControl(ArduCam::RANGE,MAX_DISTANCE);
    ArduCam::CameraInfo tofFormat = tof.getCameraInfo();

    int16_t *raw_ptr;
    int cols = tofFormat.height;
    int rows = tofFormat.width * 4;
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);

    for (;;)
    {
        frame = tof.requestFrame(200);
        if (frame != nullptr)
        {
             raw_ptr = (int16_t*)frame->getData(ArduCam::RAW_FRAME);
             cv::Mat raw_frame(cols, rows, CV_16S, raw_ptr);
             raw_frame.convertTo(raw_frame,CV_32F);
             cv::imshow("preview", raw_frame);

            if (cv::waitKey(1) == 27)
                break;
        }
        tof.releaseFrame(frame);
    }

    if (tof.stop())
        exit(-1);
    return 0;
}

#include "ArducamTOFCamera.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iostream>

using namespace Arducam;
#define MAX_DISTANCE 4

int main()
{
    ArducamTOFCamera tof;
    ArducamFrameBuffer *frame;
    if (tof.open(Connection::CSI)){
        std::cerr<<"initialization failed"<<std::endl;
        exit(-1);
    }
    if (tof.start(FrameType::RAW_FRAME)){
        std::cerr<<"Failed to start camera"<<std::endl;
        exit(-1);
    }
    //  Modify the range also to modify the MAX_DISTANCE
    tof.setControl(ControlID::RANGE,MAX_DISTANCE);
    CameraInfo tofFormat = tof.getCameraInfo();

    int16_t *raw_ptr;
    int cols = tofFormat.height;
    int rows = tofFormat.width * 4;
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);

    for (;;)
    {
        frame = tof.requestFrame(200);
        if (frame != nullptr)
        {
             raw_ptr = (int16_t*)frame->getData(FrameType::RAW_FRAME);
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
    if (tof.close())
        exit(-1);
    return 0;
}

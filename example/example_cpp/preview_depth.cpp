#include "ArduCamTOFCamera.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <iostream>

#define MAX_DISTANCE 4

void display_fps(void)
{
    static int count = 0;
    ++count;
    static std::chrono::high_resolution_clock::time_point time_beg = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::ratio<1, 1>> duration_s = time_end - time_beg;
    if (duration_s.count() >= 1)
    {
        std::cout << "fps:" << count << std::endl;
        count = 0;
        time_beg = time_end;
    }
}

void getPreview(uint8_t *preview_ptr, float *phase_image_ptr, float *amplitude_image_ptr)
{
    auto len = 240 * 180;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(phase_image_ptr + i) / MAX_DISTANCE)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
}
cv::Rect seletRect(0, 0, 0, 0);
cv::Rect followRect(0, 0, 0, 0);
void onMouse(int event, int x, int y, int flags, void *param)
{
    if (x < 4 || x > 251 || y < 4 || y > 251)
        return;
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:

        break;

    case cv::EVENT_LBUTTONUP:
        seletRect.x = x - 4 ? x - 4 : 0;
        seletRect.y = y - 4 ? y - 4 : 0;
        seletRect.width = 8;
        seletRect.height = 8;
        break;
    default:
        followRect.x = x - 4 ? x - 4 : 0;
        followRect.y = y - 4 ? y - 4 : 0;
        followRect.width = 8;
        followRect.height = 8;
        break;
    }
}

int main()
{
    ArduCam::ArduCamTOFCamera tof;
    ArduCam::ArduCamTOFFrame *frame;
    if (tof.initialize(ArduCam::DEPTH_TYPE)){
        std::cerr<<"initialization failed"<<std::endl;
        exit(-1);
    }

    if (tof.start()){
        std::cerr<<"Failed to start camera"<<std::endl;
        exit(-1);
    }
    //  Modify the range also to modify the MAX_DISTANCE value
    // tof.setMode(ArduCam::RANGE,2);
    ArduCam::FrameFormat tofFormat = tof.getFrameFormats();

    float *depth_ptr;
    float *amplitude_ptr;
    uint8_t *preview_ptr = new uint8_t[tofFormat.height * tofFormat.width];
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("preview", onMouse);

    for (;;)
    {
        frame = tof.requestFrame(200);
        if (frame != nullptr)
        {
            depth_ptr = (float *)frame->getFrameData(ArduCam::DEPTH_FRAME);
            amplitude_ptr = (float *)frame->getFrameData(ArduCam::AMPLITUDE_FRAME);
            getPreview(preview_ptr, depth_ptr, amplitude_ptr);

            cv::Mat result_frame(tofFormat.height, tofFormat.width, CV_8U, preview_ptr);
            cv::Mat depth_frame(tofFormat.height, tofFormat.width, CV_32F, depth_ptr);
            cv::Mat amplitude_frame(tofFormat.height, tofFormat.width, CV_32F, amplitude_ptr);

            cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);

            amplitude_frame.forEach<float>([](float &p, const int *position) -> void
                                           {   
                if(p < 0) p=0;
                else if(p > 255) p=255; });
            amplitude_frame.convertTo(amplitude_frame, CV_8U);
            cv::imshow("amplitude", amplitude_frame);

            cv::rectangle(result_frame, seletRect, cv::Scalar(0, 0, 0), 2);
            cv::rectangle(result_frame, followRect, cv::Scalar(255, 255, 255), 1);

            std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << std::endl;

            cv::imshow("preview", result_frame);

            if (cv::waitKey(1) == 27)
                break;
            display_fps();
        }
        tof.releaseFrame(frame);
    }

    if (tof.stop())
        exit(-1);
    return 0;
}

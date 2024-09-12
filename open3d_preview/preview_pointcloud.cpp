#include "ArducamTOFCamera.hpp"
// #include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>
#include <open3d/Open3D.h>

using namespace Arducam;

// MAX_DISTANCE value modifiable  is 2 or 4
#define MAX_DISTANCE 4000

cv::Rect seletRect(0, 0, 0, 0);
cv::Rect followRect(0, 0, 0, 0);
int max_width = 640;
int max_height = 480;
int max_range = 0;
int confidence_value = 0;
// std::atomic<int> confidence_value{0};

void on_confidence_changed(int pos, void* userdata)
{
    // confidence_value = pos;
}

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

void save_image(float* image, int width, int height)
{
    using namespace std::literals;
    // filename = "depth_$width$_$height$_f32_$time.raw"
    auto now = std::chrono::system_clock::now().time_since_epoch() / 1ms;
    std::string filename =
        "depth_" + std::to_string(width) + "_" + std::to_string(height) + "_f32_" + std::to_string(now) + ".raw";
    std::ofstream file(filename, std::ios::binary);
    file.write(reinterpret_cast<char*>(image), width * height * sizeof(float));
    file.close();
}

cv::Mat matRotateClockWise180(cv::Mat src)
{
    if (src.empty()) {
        std::cerr << "RorateMat src is empty!";
    }

    flip(src, src, 0);
    flip(src, src, 1);
    return src;
}

void getPreview(cv::Mat preview_ptr, cv::Mat amplitude_image_ptr)
{
    auto len = preview_ptr.rows * preview_ptr.cols;
    for (int line = 0; line < preview_ptr.rows; line++) {
        for (int col = 0; col < preview_ptr.cols; col++) {
            if (amplitude_image_ptr.at<float>(line, col) < confidence_value)
                preview_ptr.at<uint8_t>(line, col) = 255;
        }
    }
}

void getPreviewRGB(cv::Mat preview_ptr, cv::Mat amplitude_image_ptr)
{
    preview_ptr.setTo(cv::Scalar(0, 0, 0), amplitude_image_ptr < confidence_value);
    // cv::GaussianBlur(preview_ptr, preview_ptr, cv::Size(7, 7), 0);
}

void onMouse(int event, int x, int y, int flags, void* param)
{
    if (x < 4 || x > (max_width - 4) || y < 4 || y > (max_height - 4))
        return;
    switch (event) {
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
    ArducamTOFCamera tof;
    ArducamFrameBuffer* frame;
    if (tof.open(Connection::CSI, 0)) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }

    if (tof.start(FrameType::DEPTH_FRAME)) {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }
    //  Modify the range also to modify the MAX_DISTANCE
    tof.setControl(CameraCtrl::RANGE, MAX_DISTANCE);
    tof.getControl(CameraCtrl::RANGE, &max_range);
    auto info = tof.getCameraInfo();
    std::cout << "open camera with (" << info.width << "x" << info.height << ")" << std::endl;

    uint8_t* preview_ptr = new uint8_t[info.width * info.height * 2];
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("preview", onMouse);
    // if (info.device_type == Arducam::DeviceType::DEVICE_VGA) {
    //     // only vga support confidence
    //     cv::createTrackbar("confidence", "preview", NULL, 255, on_confidence_changed);
    // }

    float fx = max_width / (2 * tan(0.5 * M_PI * 64.3 / 180));  // 640 / 2 / tan(0.5*64.3)
    float fy = max_height / (2 * tan(0.5 * M_PI * 50.4 / 180)); // 480 / 2 / tan(0.5*50.4)
    float cx = max_width / 2;
    float cy = max_height / 2;

    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m << 1, 0, 0, 0, 
         0, -1, 0, 0,
         0, 0, -1, 0,
         0, 0, 0, 1;

    open3d::camera::PinholeCameraIntrinsic camera_intrinsic(max_width, max_height, fx, fy, cx, cy);
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("Point Cloud", max_width, max_height);
    bool first = true;

    for (;;) {
        Arducam::FrameDataFormat format;
        frame = tof.requestFrame(200);
        if (frame == nullptr) {
            continue;
        }
        frame->getFrameDataFormat(FrameType::DEPTH_FRAME, format);
        std::cout << "frame: (" << format.width << "x" << format.height << ")" << std::endl;
        max_height = format.height;
        max_width = format.width;

        float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
        float* confidence_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);
        // getPreview(preview_ptr, depth_ptr, confidence_ptr);

        cv::Mat result_frame(format.height, format.width, CV_8U, preview_ptr);
        cv::Mat depth_frame(format.height, format.width, CV_32F, depth_ptr);
        cv::Mat confidence_frame(format.height, format.width, CV_32F, confidence_ptr);

        // depth_frame = matRotateClockWise180(depth_frame);
        // result_frame = matRotateClockWise180(result_frame);
        // confidence_frame = matRotateClockWise180(confidence_frame);
        depth_frame.convertTo(result_frame, CV_8U, 255.0 / 7000, 0);

        cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_RAINBOW);
        getPreviewRGB(result_frame, confidence_frame);

        // confidence_frame.convertTo(confidence_frame, CV_8U, 255.0 / 1024, 0);

        // cv::imshow("confidence", confidence_frame);

        cv::rectangle(result_frame, seletRect, cv::Scalar(0, 0, 0), 2);
        cv::rectangle(result_frame, followRect, cv::Scalar(255, 255, 255), 1);

        std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << std::endl;

        cv::imshow("preview", result_frame);
        depth_frame.setTo(0, confidence_frame < confidence_value);
        auto depth_image = open3d::geometry::Image();
        depth_image.Prepare(max_width, max_height, 1, 4);

        memcpy(depth_image.data_.data(), depth_frame.data, depth_image.data_.size());

        auto curr_pcd = open3d::geometry::PointCloud::CreateFromDepthImage(
                depth_image, camera_intrinsic, Eigen::Matrix4d::Identity(), 5000.0, 5000.0);
        
        // memcpy(pcd->points_.data(), curr_pcd->points_.data(), pcd->points_.size());
        pcd->points_ = curr_pcd->points_;

        pcd->Transform(m);

        if (first) {
            vis.AddGeometry(pcd);
            first = false;
        }

        // auto voxel_down_pcd = pcd->VoxelDownSample(20);
        // auto tuple = voxel_down_pcd->RemoveStatisticalOutliers(20, 2.0);
        // auto inlier_cloud = voxel_down_pcd->SelectByIndex(std::get<std::vector<size_t>>(tuple));
        // pcd->points_ = inlier_cloud->points_;
        vis.UpdateGeometry(pcd);
        vis.PollEvents();
        vis.UpdateRender();

        auto key = cv::waitKey(1);
        if (key == 27 || key == 'q') {
            break;
        } else if (key == 's') {
            save_image(depth_ptr, format.width, format.height);
        }
        display_fps();
        tof.releaseFrame(frame);
    }

    vis.DestroyVisualizerWindow();

    if (tof.stop()) {
        return -1;
    }

    if (tof.close()) {
        return -1;
    }

    return 0;
}

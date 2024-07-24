#include "ArducamTOFCamera.hpp"
#include <ctime>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#if 0
#include <boost/signals2/mutex.hpp>
#include <boost/thread/thread.hpp>
#endif

using namespace Arducam;

int max_width = 240;
int max_height = 180;
int max_range = 0;

int confidence_value = 30;
void on_confidence_changed(int pos, void* userdata)
{
    //
}

#if 0
boost::mutex updateModelMutex;
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addCoordinateSystem(1);
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }
}
#else
pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addCoordinateSystem(100);
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}
#endif

void getPreview(uint8_t* preview_ptr, float* depth_image_ptr, float* amplitude_image_ptr)
{
    const auto len = max_width * max_height;
    for (int i = 0; i < len; i++) {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(depth_image_ptr + i) * 1000.f / max_range)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
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

int main(int argc, char* argv[])
{
    ArducamTOFCamera tof;
    if (argc > 1) {
        const char* cfg_path = argv[1];
        if (tof.openWithFile(cfg_path, 375e5)) {
            std::cerr << "Failed to open camera with cfg: " << cfg_path << std::endl;
            return -1;
        }
    } else {
        if (tof.open(Connection::CSI)) {
            std::cerr << "initialization failed" << std::endl;
            return -1;
        }
    }

    if (1) {
        tof.setControl(CameraCtrl::FMT_WIDTH, 640);
        tof.setControl(CameraCtrl::FMT_HEIGHT, 480);
        tof.setControl(CameraCtrl::MODE, to_int(TofWorkMode::DOUBLE_FREQ));
        tof.setControl(CameraCtrl::FRAME_MODE, to_int(TofFrameWorkMode::DOUBLE_FREQ_4PHASE_GRAY_4PHASE_BG));
    } else {
        tof.setControl(CameraCtrl::FMT_WIDTH, 640);
        tof.setControl(CameraCtrl::FMT_HEIGHT, 480);
        tof.setControl(CameraCtrl::MODE, to_int(TofWorkMode::SINGLE_FREQ));
        tof.setControl(CameraCtrl::FRAME_MODE, to_int(TofFrameWorkMode::SINGLE_FREQ_4PHASE_GRAY));
    }

    if (tof.start(FrameType::DEPTH_FRAME)) {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }

    tof.getControl(CameraCtrl::RANGE, &max_range);
    auto info = tof.getCameraInfo();
    std::cout << "open camera with (" << info.width << "x" << info.height << ") with range " << max_range << std::endl;
    // double x = 0, y = 0, z = 0, dx = 0, dy = 0, dz = 0;

    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("confidence", "preview", &confidence_value, 255, on_confidence_changed);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // cloud_ptr->width = num_points;
    // cloud_ptr->height = 1;
    // cloud_ptr->is_dense = true;
    // cloud_ptr->resize(num_points);
    vtkObject::GlobalWarningDisplayOff();
#if 0
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud_ptr);
    boost::thread vthread(&viewerRunner, viewer);
#else
    pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud_ptr);
#endif
    for (;;) {
        ArducamFrameBuffer* frame = tof.requestFrame(200);
        if (frame != nullptr) {
            float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
            float* amplitude_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);

            Arducam::FrameDataFormat format;
            frame->getFrameDataFormat(FrameType::DEPTH_FRAME, format);
            std::cout << "frame: (" << format.width << "x" << format.height << ")" << std::endl;
            max_height = format.height;
            max_width = format.width;

            constexpr auto max_width = 640;
            constexpr auto max_height = 480;
            constexpr float fx = max_width / (2 * tan(0.5 * M_PI * 64.3 / 180));  // 640 / 2 / tan(0.5*64.3)
            constexpr float fy = max_height / (2 * tan(0.5 * M_PI * 50.4 / 180)); // 480 / 2 / tan(0.5*50.4)
            // const float fx = 509.128;
            // const float fy = 509.334;
            // getPreview(preview_ptr, depth_ptr, amplitude_ptr);

            cv::Mat result_frame(format.height, format.width, CV_8U);
            cv::Mat depth_frame(format.height, format.width, CV_32F, depth_ptr);
            cv::Mat amplitude_frame(max_height, max_width, CV_32F, amplitude_ptr);

            depth_frame.convertTo(result_frame, CV_8U, 255.0 / max_range, 0);
            getPreview(result_frame, amplitude_frame);
            cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_RAINBOW);

            // cv::resize(result_frame, result_frame, cv::Size(720, 540));
            cv::imshow("preview", result_frame);
            cloud_ptr->clear();
            unsigned long int pos = 0;
            for (int row_idx = 0; row_idx < max_height; row_idx++)
                for (int col_idx = 0; col_idx < max_width; col_idx++, pos++) {
                    if (amplitude_ptr[pos] > confidence_value) {
                        float zz = depth_ptr[pos];
                        float xx = ((((max_width / 2) - col_idx)) / fx) * zz;
                        float yy = (((max_height / 2) - row_idx) / fy) * zz;
                        pcl::PointXYZ ptemp(xx, yy, zz);
                        cloud_ptr->points.push_back(ptemp);
                    }
                    // else
                    // {
                    //     pcl::PointXYZ ptemp(0, 0, 0);
                    //     cloud_ptr->points.push_back(ptemp);
                    // }
                }
            cloud_ptr->width = cloud_ptr->points.size();
            cloud_ptr->height = 1;
            cloud_ptr->is_dense = true;
            tof.releaseFrame(frame);
#if 0
            boost::mutex::scoped_lock updateLock(updateModelMutex);
            viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr, "sample cloud");
            updateLock.unlock();
#else
            // viewer->setCameraPosition(x, y, z, dx, dy, dz, 0, 0, 0);
            viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr, "sample cloud");
            try {
                viewer->spinOnce(50);
            } catch (...) {
                std::cerr << __FILE__ << '[' << __LINE__ << "]: " << __FUNCTION__
                          << "viewer->spinOnce(100); Segmentation fault,You can refer to "
                             "https://github.com/PointCloudLibrary/pcl/issues/5189 to solve the problem"
                          << std::endl;
                viewer->spin();
            }
#endif
            int key = cv::waitKey(1);
            // std::cout << "key: " << key << std::endl;
            switch (key) {
                // case 'w': {
                //     z += 0.1;
                //     break;
                // }
                // case 'a': {
                //     x += 0.1;
                //     break;
                // }
                // case 's': {
                //     z -= 0.1;
                //     break;
                // }
                // case 'd': {
                //     x -= 0.1;
                //     break;
                // }
                // case 'q': {
                //     y -= 0.1;
                //     break;
                // }
                // case 'e': {
                //     y += 0.1;
                //     break;
                // }
                // case 'z': {
                //     x = 0;
                //     y = 0;
                //     z = 0;
                //     break;
                // }
                // case 82: {
                //     dy += 0.1 * M_PI / 180;
                //     break;
                // }
                // case 81: {
                //     dx += 0.1 * M_PI / 180;
                //     break;
                // }
                // case 84: {
                //     dy -= 0.1 * M_PI / 180;
                //     break;
                // }
                // case 83: {
                //     dx -= 0.1 * M_PI / 180;
                //     break;
                // }

            case 'p': {
                char buff[60];
                std::time_t t = std::time(0);
                tm* nowtime = localtime(&t);
                sprintf(buff, "image_%04d%02d%02d-%02d%02d%02d.png", 1900 + nowtime->tm_year, nowtime->tm_mon + 1,
                        nowtime->tm_mday, nowtime->tm_hour + 1, nowtime->tm_min + 1, nowtime->tm_sec + 1);
                cv::imwrite(buff, result_frame);
                std::cout << "save image! file: '" << buff << "'" << std::endl;
                break;
            }
            case 's': {
                char buff[60];
                std::time_t t = std::time(0);
                tm* nowtime = localtime(&t);
                sprintf(buff, "sensor_%04d%02d%02d-%02d%02d%02d.pcd", 1900 + nowtime->tm_year, nowtime->tm_mon + 1,
                        nowtime->tm_mday, nowtime->tm_hour + 1, nowtime->tm_min + 1, nowtime->tm_sec + 1);
                pcl::io::savePCDFileASCII(buff, *cloud_ptr);
                std::cout << "save pcd! file: '" << buff << "'" << std::endl;
                break;
            }
            case 'q':
            case 27: {
                goto exit_main;
                break;
            }
            default: {
                // std::cout << "key: " << key << std::endl;
                break;
            }
            }
        }
    }
exit_main:
    cv::destroyAllWindows();
#if 0
    viewer->close();
    vthread.join();
#else
    viewer->close();
    // runner.join();
#endif

    tof.stop();

    tof.close();

    return 0;
}

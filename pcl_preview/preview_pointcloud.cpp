
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#if 0
#include <boost/signals2/mutex.hpp>
#include <boost/thread/thread.hpp>
#endif
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ctime>
#include "ArducamTOFCamera.hpp"

#define MAX_DISTANCE 4
using namespace Arducam;

#if 0
boost::mutex updateModelMutex;
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addCoordinateSystem(0.01);
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
pcl::visualization::PCLVisualizer simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(0.01);
    viewer.setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer.initCameraParameters();
    return (viewer);
}
#endif

void getPreview(uint8_t *preview_ptr, float *depth_image_ptr, float *amplitude_image_ptr)
{
    auto len = 240 * 180;
    for (int i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(depth_image_ptr + i) / MAX_DISTANCE)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
}

int main()
{
    ArducamTOFCamera tof;
    ArducamFrameBuffer *frame;
    char buff[60];
    float *depth_ptr;
    float *amplitude_ptr;
    uint8_t *preview_ptr = new uint8_t[43200];
    if (tof.open(Connection::CSI))
    {
        std::cerr << "initialization failed" << std::endl;
        exit(-1);
    }
    if (tof.start())
    {
        std::cerr << "Failed to start camera" << std::endl;
        exit(-1);
    }
    tof.setControl(CameraCtrl::RANGE, MAX_DISTANCE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_ptr->points.push_back(pcl::PointXYZ(10, 10, 4));
    cloud_ptr->width = cloud_ptr->size();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = true;
    vtkObject::GlobalWarningDisplayOff();
#if 0
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud_ptr);
    boost::thread vthread(&viewerRunner, viewer);
#else
    pcl::visualization::PCLVisualizer viewer = simpleVis(cloud_ptr);
#endif
    const float fx = 240 / (2 * tan(0.5 * M_PI * 64.3 / 180));
    const float fy = 180 / (2 * tan(0.5 * M_PI * 50.4 / 180));
    for (;;)
    {
        frame = tof.requestFrame(200);
        if (frame != nullptr)
        {
            depth_ptr = (float *)frame->getData(FrameType::DEPTH_FRAME);
            amplitude_ptr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);
            getPreview(preview_ptr, depth_ptr, amplitude_ptr);
            cv::Mat result_frame(180, 240, CV_8U, preview_ptr);
            cv::Mat amplitude_frame(180, 240, CV_32F, amplitude_ptr);

            cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);

            cv::resize(result_frame, result_frame, cv::Size(720, 540));
            cv::imshow("preview", result_frame);
            cloud_ptr->clear();
            unsigned long int pos = 0;
            for (int row_idx = 0; row_idx < 180; row_idx++)
                for (int col_idx = 0; col_idx < 240; col_idx++, pos++)
                {
                    if (amplitude_ptr[pos] > 30)
                    {
                        float zz = depth_ptr[pos];

                        float xx = (((120 - col_idx)) / fx) * zz;
                        float yy = ((90 - row_idx) / fy) * zz;
                        pcl::PointXYZ ptemp(xx, yy, zz);
                        cloud_ptr->points.push_back(ptemp);
                    }
                    else
                    {
                        pcl::PointXYZ ptemp(0, 0, 0);
                        cloud_ptr->points.push_back(ptemp);
                    }
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
            viewer.updatePointCloud<pcl::PointXYZ>(cloud_ptr, "sample cloud");
            viewer.spinOnce(100);
#endif

            switch (cv::waitKey(1))
            {
            case 's':
            {
                std::time_t t = std::time(0);
                tm *nowtime = localtime(&t);
                sprintf(buff, "image_%d%d%d%d%d%d.png", 1900 + nowtime->tm_year, nowtime->tm_mon + 1, nowtime->tm_mday, nowtime->tm_hour + 1, nowtime->tm_min + 1, nowtime->tm_sec + 1);
                cv::imwrite(buff, result_frame);
                std::cout << "save image!" << std::endl;
                break;
            }
            case 'q':
                goto exit_main;
                break;
            case 'd':
            {
                std::time_t t = std::time(0);
                tm *nowtime = localtime(&t);
                sprintf(buff, "sensor_%d%d%d%d%d%d.pcd", 1900 + nowtime->tm_year, nowtime->tm_mon + 1, nowtime->tm_mday, nowtime->tm_hour + 1, nowtime->tm_min + 1, nowtime->tm_sec + 1);
                pcl::io::savePCDFileASCII(buff, *cloud_ptr);
                std::cout << "save pcd!" << std::endl;
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
    viewer.close();
#endif
    tof.stop();
    tof.close();
    return 0;
}

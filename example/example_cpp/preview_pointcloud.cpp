
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/signals2/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "ArduCamTOFCamera.hpp"

#define MAX_DISTANCE 2

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
    ArduCam::ArduCamTOFCamera tof;
    ArduCam::NodeData *frame;
    if (tof.initialize(ArduCam::DEPTH_TYPE))
        exit(-1);
    if (tof.start())
        exit(-1);
    tof.setControl(ArduCam::RANGE,2);

    float *depth_ptr;
    float *amplitude_ptr;
    uint8_t *preview_ptr = new uint8_t[180 * 240];
    unsigned long int ponitsizes = 180 * 240;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &pcloud = *cloud_ptr;

    pcloud.points.push_back(pcl::PointXYZ(10, 10, 4));
    pcloud.width = cloud_ptr->size();
    pcloud.height = 1;
    pcloud.is_dense = true;
    vtkObject::GlobalWarningDisplayOff();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud_ptr);
    boost::thread vthread(&viewerRunner, viewer);
    int cnt = 0;
    char buff[60];

    for (;;)
    {
        frame = tof.requestFrame(200);

        if (frame != nullptr)
        {
            depth_ptr = (float *)frame->getData(ArduCam::DEPTH_FRAME);
            amplitude_ptr = (float *)frame->getData(ArduCam::AMPLITUDE_FRAME);
            getPreview(preview_ptr, depth_ptr, amplitude_ptr);
            cv::Mat result_frame(180, 240, CV_8U, preview_ptr);
            cv::Mat amplitude_frame(180, 240, CV_32F, amplitude_ptr);

            cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_JET);

            cv::resize(result_frame, result_frame, cv::Size(960, 720));
            cv::imshow("preview", result_frame);
            pcloud.clear();

            for (unsigned long int _row = 0; _row < ponitsizes; _row++)
            {
                if (amplitude_ptr[_row] > 30)
                {
                    float fx = 240 / (2 * tan(0.5 * M_PI * 64.3 / 180));
                    float fy = 180 / (2 * tan(0.5 * M_PI * 50.4 / 180));
                    int vy = _row / 240;
                    int ux = _row % 240;
                    float zz = depth_ptr[_row]; // fmod(depth_ptr[_row] , 4);

                    float x = (((240 - ux - 120)) / fx) * zz;
                    float y = ((180 - vy - 90) / fy) * zz;
                    float z = zz;
                    pcl::PointXYZ ptemp(x, y, z);
                    pcloud.points.push_back(ptemp);
                }
                else
                {
                    pcl::PointXYZ ptemp(0, 0, 0);
                    pcloud.points.push_back(ptemp);
                }
            }
            pcloud.width = pcloud.points.size();
            pcloud.height = 1;
            pcloud.is_dense = false;
            boost::mutex::scoped_lock updateLock(updateModelMutex);
            viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr, "sample cloud");
            updateLock.unlock();
            boost::this_thread::sleep(boost::posix_time::microseconds(100));
            switch (cv::waitKey(1))
            {
            case 's':
                cnt++;
                sprintf(buff, "image_%d.png", cnt);
                cv::imwrite(buff, result_frame);
                std::cout << "save image!" << std::endl;
                break;
            case 'q':
                cv::destroyAllWindows();
                viewer->close();
                vthread.join();
                tof.stop();
                exit(0);
                break;
            case 'd':
                cnt++;
                sprintf(buff, "sensor_%d.pcd", cnt);
                pcl::io::savePCDFileASCII (buff, pcloud);
                std::cout << "save pcd!" << std::endl;
            }
        }
        tof.releaseFrame(frame);
    }
    return 0;
}

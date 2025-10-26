#ifndef CAMERA_H
#define CAMERA_H

#include "ArducamTOFCamera.hpp"

class Camera {
public:
    Camera(Arducam::Connection type, int index = 0);
    ~Camera();

    bool open();
    bool start(Arducam::FrameType type);
    void stop();
    void close();

    Arducam::ArducamFrameBuffer* requestFrame(int timeout);
    void releaseFrame(Arducam::ArducamFrameBuffer* frame);
    void setControl(Arducam::Control id, int value);
    Arducam::CameraInfo getCameraInfo();

private:
    Arducam::ArducamTOFCamera tof_;
    Arducam::Connection connection_type_;
    int device_index_;
    Arducam::FrameType frame_type_;
    bool is_open_ = false;
    bool is_running_ = false;
};

#endif // CAMERA_H

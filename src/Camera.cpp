#include "Camera.h"
#include <iostream>

Camera::Camera(Arducam::Connection type, int index) : connection_type_(type), device_index_(index) {}

Camera::~Camera() {
    if (is_running_) {
        stop();
    }
    if (is_open_) {
        close();
    }
}

bool Camera::open() {
    if (tof_.open(connection_type_, device_index_) != 0) {
        std::cerr << "initialization failed" << std::endl;
        return false;
    }
    is_open_ = true;
    return true;
}

bool Camera::start(Arducam::FrameType type) {
    if (tof_.start(type) != 0) {
        std::cerr << "Failed to start camera" << std::endl;
        return false;
    }
    frame_type_ = type;
    is_running_ = true;
    return true;
}

void Camera::stop() {
    if (tof_.stop() != 0) {
        std::cerr << "Failed to stop camera" << std::endl;
    }
    is_running_ = false;
}

void Camera::close() {
    if (tof_.close() != 0) {
        std::cerr << "Failed to close camera" << std::endl;
    }
    is_open_ = false;
}

Arducam::ArducamFrameBuffer* Camera::requestFrame(int timeout) {
    return tof_.requestFrame(timeout);
}

void Camera::releaseFrame(Arducam::ArducamFrameBuffer* frame) {
    tof_.releaseFrame(frame);
}

void Camera::setControl(Arducam::Control id, int value) {
    tof_.setControl(id, value);
}

Arducam::CameraInfo Camera::getCameraInfo() {
    return tof_.getCameraInfo();
}

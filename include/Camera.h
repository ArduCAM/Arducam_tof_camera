#ifndef CAMERA_H
#define CAMERA_H

#include "ArducamTOFCamera.hpp"

/**
 * @brief Represents a wrapper around the Arducam TOF camera SDK.
 *
 * This class simplifies the interaction with the Arducam TOF camera by
 * providing a high-level interface for opening, configuring, and capturing
 * frames from the device.
 */
class Camera {
public:
    /**
     * @brief Construct a new Camera object.
     *
     * @param type The connection type of the camera (e.g., USB or CSI).
     * @param index The device index of the camera.
     */
    Camera(Arducam::Connection type, int index = 0);
    /**
     * @brief Destroy the Camera object.
     */
    ~Camera();

    /**
     * @brief Opens the connection to the camera.
     *
     * @return true if the camera was opened successfully, false otherwise.
     */
    bool open();
    /**
     * @brief Starts the camera's frame capture.
     *
     * @param type The type of frame to capture (e.g., depth or amplitude).
     * @return true if the capture was started successfully, false otherwise.
     */
    bool start(Arducam::FrameType type);
    /**
     * @brief Stops the camera's frame capture.
     */
    void stop();
    /**
     * @brief Closes the connection to the camera.
     */
    void close();

    /**
     * @brief Requests a frame from the camera.
     *
     * @param timeout The maximum time to wait for a frame, in milliseconds.
     * @return Arducam::ArducamFrameBuffer* A pointer to the frame buffer, or nullptr if a timeout occurs.
     */
    Arducam::ArducamFrameBuffer* requestFrame(int timeout);
    /**
     * @brief Releases a previously requested frame buffer.
     *
     * @param frame A pointer to the frame buffer to release.
     */
    void releaseFrame(Arducam::ArducamFrameBuffer* frame);
    /**
     * @brief Sets a camera control value.
     *
     * @param id The ID of the control to set.
     * @param value The value to set for the control.
     */
    void setControl(Arducam::Control id, int value);
    /**
     * @brief Gets the camera's information.
     *
     * @return Arducam::CameraInfo A struct containing the camera's information.
     */
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

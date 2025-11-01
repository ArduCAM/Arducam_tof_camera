#ifndef TOFAPPLICATION_H
#define TOFAPPLICATION_H

#include "Camera.h"
#include "Viewer.h"
#include <memory>

/**
 * @brief Manages the main application logic for the TOF camera.
 *
 * This class is responsible for initializing the camera and viewer,
 * and running the main application loop to capture and display frames.
 */
class TOFApplication {
public:
    /**
     * @brief Construct a new TOFApplication object.
     */
    TOFApplication();
    /**
     * @brief Destroy the TOFApplication object.
     */
    ~TOFApplication();

    /**
     * @brief Initializes the application, including the camera and viewer.
     *
     * @return int Returns 0 on success, -1 on failure.
     */
    int initialize();
    /**
     * @brief Runs the main application loop.
     */
    void run();

private:
    std::unique_ptr<Camera> camera_;
    std::unique_ptr<Viewer> viewer_;
};

#endif // TOFAPPLICATION_H

#ifndef __TOF_CAMERA_H
#define __TOF_CAMERA_H

#include <unordered_map>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <math.h>
#include <thread>
#include <mutex>
#include <map>

#include "ArduCamTOFSensor.hpp"
#include "ArduCamTOFFrame.hpp"
#include "Semaphore.hpp"
namespace ArduCam
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    /**
     * @brief
     *
     */
    typedef std::function<void(std::string frame_type, void *frame_ptr)> DoneCallback;
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

    /**
     * @struct CameraInfo
     * @brief Basic information of the camera module
     */
    struct CameraInfo
    {
        int cameraId;
        std::string cameraPath;
        FrameFormat frameInfos;
        // std::string connecting;
        unsigned int width;
        unsigned int height;
        unsigned long int imageSize;
    };

    typedef enum {
    	RANGE,
        EXPOSURE,
    } CameraMode;

    /**
     * @brief Camera application layer class, used to manage the camera and process frame data
     *
     */
    class ArduCamTOFCamera
    {
#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
 * @brief Convert data stored in little endian to int16 data
 *
 */
#define CONVERTTORAW(x, y) (((int16_t)(((((uint16_t)(x)) << 8) | (0xff & (y))) << 5)) >> 1)
#endif
    public:
        /**
         * @brief Initialize the camera configuration and turn on the camera, set the initialization frame according to the @ref type
         *
         * @param type Specify the camera output format.
         *      This parameter can be one of the following values:
         *          @arg RAW_TYPE
         *          @arg DEPTH_TYPE
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int initialize(OutputType type);
        /**
         * @brief Start the camera stream and start processing.
         *
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int start();
        /**
         * @brief Stop camera stream and processing.
         *
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int stop();
        /**
         * @brief  Specifies the frame Output Type
         * @note Please call the function operation to reset the frame output format before starting the camera
         *
         * @param type Specify the camera output format.
         *      This parameter can be one of the following values:
         *          @arg RAW_TYPE
         *          @arg DEPTH_TYPE
         *
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int setOutputType(OutputType type);

        /**
         * @brief Set camera parameters
         *
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int setControl(CameraMode mode,int value);

        /**
         * @brief Get the Camera frames format.
         *
         * @return All frame data formats contained in frame, The returned value include: width, height and Frametype
         */
        FrameFormat getFrameFormats() const;

        /**
         * @brief Request a frame of data from the frame processing thread
         *
         * @param timeout Timeout time, -1 means to wait all the time, 0 means immediate range, other values indicate the maximum waiting time, the unit is milliseconds.
         * @return ArduCamTOFFrame class address.
         */
        ArduCamTOFFrame *requestFrame(int16_t timeout);

        /**
         * @brief Free the memory space of the frame
         *
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int releaseFrame(ArduCamTOFFrame *);
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    private:
        CameraInfo m_Info;

        std::shared_ptr<ArduCamTOFSensor> m_depthSensor;

        std::shared_ptr<ArduCamTOFFrame> m_frame;

        std::thread capture_thread_;

        bool capture_thread_abort_;

        std::mutex mutex_;

        Semaphore sem_;

        float f_mod = 75e6;       //! Camera light wave modulation frequency

        const float c = 3e8;      //! speed of light
    
        const float pi = M_PI; //! π

    private:
        void getPhaseAndAmplitudeImage(uint8_t *cache_ptr, float *phase_ptr, float *amplitude_ptr);

        void getPhaseAndAmplitudeImage(int16_t *raw_ptr, float *phase_ptr, float *amplitude_ptr);

        // void getPreview(uint8_t *preview_ptr, float *phase_ptr, float *amplitude_ptr);

        void getRawImages(uint8_t *cache_ptr, int16_t *raw_ptr);

        int analysisFrame(ArduCamTOFFrame *frame);

        void captureThread();

        int setCameraRange(int value);

    public:
        ArduCamTOFCamera();

        ~ArduCamTOFCamera();
#endif
    };
} // namespace ArduCam
#endif
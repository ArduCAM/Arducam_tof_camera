#ifndef __TOF_UNITY_H
#define __TOF_UNITY_H

#include <string>
#include <vector>
namespace ArduCam
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS

#define ARDUCAM_TOF_SENSOR_BUFFER_COUNT 4

#define ARDUCAM_TOF_SENSOR_CACHE_TIMEOUT 5000

#define ARDUCAM_TOF_SENSOR_CACHE_BUFFER_COUNT 8

#endif
    /**
     * @brief Some types of frame data
     *
     */
    typedef enum
    {
        RAW_FRAME = 0,
        AMPLITUDE_FRAME,
        DEPTH_FRAME,
        CACHE_FRAME,
        FRAME_TYPE_COUNT,
    } FrameType;

    /**
     * @brief Description of frame data format
     *
     */
    typedef struct
    {
        unsigned int width;  //! width of frame
        unsigned int height; //! height of frame
        FrameType type;      //! type of frame
        uint8_t bitdepth;
    } FrameDataFormat;

    /**
     * @brief The output type of the frame, the RAW type outputs RAW_FRAME, the DEPTH type outputs DEPTH_FRAME and AMPLITUDE_FRAME
     *
     */
    typedef enum
    {
        RAW_TYPE = 0,
        DEPTH_TYPE,
        OUTPUT_TYPE_COUNT,
    } OutputType;

    /**
     * @brief Frame description for the frame data set
     *
     */
    typedef struct
    {
        OutputType type;
        std::vector<FrameDataFormat> dataFormats;
        unsigned int width;
        unsigned int height;
    } FrameFormat;

    /**
     * @struct CameraInfo
     * @brief Basic information of the camera module
     */
    struct CameraInfo
    {
        int cameraId;
        char cameraPath[80];
        FrameFormat frameInfos;
        // std::string connecting;
        unsigned int width;
        unsigned int height;
        unsigned long int imageSize;
    };

    typedef enum
    {
        RANGE,
        EXPOSURE,
    } CameraMode;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
    bool operator==(const FrameDataFormat &lhs, const FrameDataFormat &rhs);

    bool operator!=(const FrameDataFormat &lhs, const FrameDataFormat &rhs);

    bool operator==(const FrameFormat &lhs, const FrameFormat &rhs);

    bool operator!=(const FrameFormat &lhs, const FrameFormat &rhs);
#endif
} // namespace ArduCam
#endif
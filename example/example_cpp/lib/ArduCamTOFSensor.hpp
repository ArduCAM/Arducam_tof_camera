#ifndef __TOF_SENSOR_H
#define __TOF_SENSOR_H

#include <cstdint>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cstring>

namespace ArduCam
{
#ifndef DOXYGEN_SHOULD_SKIP_THIS

#define ARDUCAM_TOF_SENSOR_BUFFER_COUNT 4

#define ARDUCAM_TOF_SENSOR_CACHE_TIMEOUT 5000

#define ARDUCAM_TOF_SENSOR_CACHE_BUFFER_COUNT 8

#endif
    /**
     * @brief Direct operation class for camera
     * 
     */
    class ArduCamTOFSensor
    {
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
    private:
        int tofd;

        uint8_t *video_buffer_ptr[ARDUCAM_TOF_SENSOR_CACHE_BUFFER_COUNT];

    private:
        inline int xioctl(int fh, int request, void *arg);

    public:
        ArduCamTOFSensor();

        ~ArduCamTOFSensor();
    #endif
    public:
        /**
         * @brief Open the camera specified by index and video
         *
         * @param index Device index.
         * @param video Device file description path.
         *
         * @return Return Status code.
         */
        int open(int index, std::string video);
        
        /**
         * @brief Close camera
         *
         * @return Return Status code.
         */
        int close();

        /**
         * @brief Start the camera stream.
         *
         * @param width of the stream.
         * @param height of the stream.
         *
         * @return Return Status code.
         */

        int start(int width, int height);
        /**
         * @brief Stop camera stream.
         *
         * @return Return Status code.
         */
        int stop();

        /**
         * @brief Read frame data from the camera.
         *
         * @param data_ptr  Address of the frame data
         * @param timestamp The timestamp of the frame.
         *
         * @return Return Status code.
         */
        int getFrame(uint8_t *data_ptr, struct timeval &timestamp);
        /**
         * @brief Switch camera range.
         * 
         * @param value Mode value,
         * This parameter can be one of the following values:
         *          @arg  0 is 4m range mode
         *          @arg 1 is 2m range mode
         * @return Return Status code. 
         */
        int setMode(int value);
        /**
         * @brief Set camera exposure time.
         * 
         * @param value exposure time.The value ​​range is 1 to 65523
         * @return Return Status code.  
         */
        int setExposure(int value);
    };

} // namespace ArduCam

#endif

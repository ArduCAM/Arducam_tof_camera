#include "ArduCamDepthCamera.h"
#include <stdlib.h>
#include <stdio.h>

void getPreview(uint8_t *preview_ptr, float *phase_image_ptr, float *amplitude_image_ptr)
{
    unsigned long int len = 240 * 180;
    for (unsigned long int i = 0; i < len; i++)
    {
        uint8_t amplitude = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float phase = ((1 - (*(phase_image_ptr + i) / 2)) * 255);
        uint8_t depth = phase > 255 ? 255 : phase;
        *(preview_ptr + i) = depth & amplitude;
    }
}

int main()
{
    ArducamDepthCamera tof = createArducamDepthCamera();
    FrameBuffer frame;
    if (init(tof,USB,DEPTH_TYPE,0))
        exit(-1);
    if (start(tof))
        exit(-1);
    uint8_t *preview_ptr = malloc(180*240*sizeof(uint8_t)) ;
    float* depth_ptr = 0;
    int16_t *raw_ptr = 0;
    float *amplitude_ptr = 0;
    FrameFormat format;
    if ((frame = requestFrame(tof,200)) != 0x00){
        format = getFormat(frame,DEPTH_FRAME);
    }
    for (;;)
    {
        if ((frame = requestFrame(tof,200)) != 0x00)
        {
            depth_ptr = (float*)getDepthData(frame);
            printf("Center distance:%.2f.\n",depth_ptr[21600]);
            amplitude_ptr = (float*)getAmplitudeData(frame);
            getPreview(preview_ptr,depth_ptr,amplitude_ptr);
            releaseFrame(tof,frame);  
        }
    }

    if (stop(tof))
        exit(-1);
    return 0;
}

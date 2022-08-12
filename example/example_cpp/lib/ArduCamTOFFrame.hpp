#ifndef __TOF_FRAME_H
#define __TOF_FRAME_H

#include "ArduCamTOFUnity.hpp"
#include "ArduCamTOFLinkList.hpp"

namespace ArduCam
{
    /**
     * @brief Memory space management for saving frame formats and frames
     *
     */
    class ArduCamTOFFrame
    {
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    private:
        FrameFormat m_Infos;
        std::unique_ptr<LinkList> m_implData;
        std::unique_ptr<CacheData> m_cache;
#endif
    public:
        struct timeval m_timestamp;
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    // private:
    //     void allocFrameData(const FrameFormat &details);

    public:
        // ArduCamTOFFrame();
#endif
        /**
         * @brief Construct a frame instance according to the frame format data and apply for memory space.
         *
         */
        ArduCamTOFFrame(const FrameFormat &);
#ifndef DOXYGEN_SHOULD_SKIP_THIS
        ~ArduCamTOFFrame();
        /**
         * @brief deep copy
         *
         */
        ArduCamTOFFrame(const ArduCamTOFFrame &);
#endif
        /**
         * @brief deep copy
         *
         * @param a_fm ArduCamTOFFrame class instance.
         */
        ArduCamTOFFrame(const std::shared_ptr<ArduCamTOFFrame> a_fm);

        // ArduCamTOFFrame &operator=(const ArduCamTOFFrame &);

    public:
        /**
         * @brief refer to @ref FrameFormat,Resetting the frame format and requesting frame memory space.
         *
         * @param details Frame format description structure, including width, height, frameType and bitDepth.
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int setFrameInfo(const FrameFormat &details);
        /**
         * @brief Get all frame data formats of the current frame.
         *
         * @param details Returns all frame data formats within the frame,If the output type is DEPTH_TYPE, it includes DEPTH_FRAME and AMPLITUDE_FRAME, if it is RAW_TYPE, only RAW_FRAME.
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int getFrameFormat(FrameFormat &details) const;
        /**
         * @brief Specify the frame data type to get the frame data format.
         *
         * @param type Specify the frame data type.
         *      This parameter can be one of the following values:
         *          @arg RAW_FRAME
         *          @arg AMPLITUDE_FRAME
         *          @arg DEPTH_FRAME
         * @param format Returns the frame data format of the specified frame type
         * @return Return Status code, The returned value can be: OK or ERROR(0 or -1).
         */
        int getFrameDataFormat(const FrameType &type, FrameDataFormat &format) const;
        /**
         * @brief Get the Pointer of Frame Data.
         *
         * @param type  Specify the frame data type.
         *      This parameter can be one of the following values:
         *          @arg RAW_FRAME
         *          @arg AMPLITUDE_FRAME
         *          @arg DEPTH_FRAME
         * @return Return Pointer of Frame Data.The returned pointer type can be one of the following type:
         *          - int16_t* : RAW_FRAME
         *          - float*   : DEPTH_FRAME
         *          - float*   : AMPLITUDE_FRAME
         */

        CacheData* getCacheData();


        NodeData* getFrameData();

        int refrashFrame(NodeData* node);


        NodeData* requestFrame();

        int realseFrame(NodeData* node);

    public:
        /**
         * @brief Get camera output type.
         *
         * @return OutputType:RAW_TYPE,DEPTH_TYPE.
         */
        inline OutputType getOutputType()
        {
            return m_Infos.type;
        }
    };
} // namespace Arducam

#endif

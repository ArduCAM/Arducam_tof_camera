#ifndef __TOF_FRAME_H
#define __TOF_FRAME_H

#include <vector>
#include <memory>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cstring>

namespace ArduCam
{
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

    #ifndef DOXYGEN_SHOULD_SKIP_THIS
    bool operator==(const FrameDataFormat &lhs, const FrameDataFormat &rhs);

    bool operator!=(const FrameDataFormat &lhs, const FrameDataFormat &rhs);

    bool operator==(const FrameFormat &lhs, const FrameFormat &rhs);

    bool operator!=(const FrameFormat &lhs, const FrameFormat &rhs);
    #endif
    /**
     * @brief Memory space management for saving frame formats and frames
     * 
     */
    class ArduCamTOFFrame
    {
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
    private:
        struct ImplData;
        FrameFormat m_Infos;
        std::unique_ptr<ImplData> m_implData;
        // std::map<std::string, std::string> m_attributes;
    #endif
    public:
        struct timeval m_timestamp;
    #ifndef DOXYGEN_SHOULD_SKIP_THIS
    private:
        void allocFrameData(const FrameFormat &details);
 
    public:
        ArduCamTOFFrame();
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
        void *getFrameData(const FrameType &type);

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

#ifndef __TOF_LINKLIST_H
#define __TOF_LINKLIST_H

#include <vector>
#include <mutex>
#include <memory>

#include "ArduCamTOFUnity.hpp"

namespace ArduCam
{

    class AbstractData
    {
    public:
        AbstractData() = default;
        virtual void *getData(FrameType type) = 0;
        virtual int getFrameDataFormat(FrameType type, FrameDataFormat &format) = 0;
        virtual ~AbstractData() = default;
    };

    class CacheData : public AbstractData
    {
        std::unique_ptr<uint8_t[]> cacheData;
        FrameDataFormat m_dataFormat;

    public:
        CacheData(FrameDataFormat dataFormat);
        void *getData(FrameType type);
        int getFrameDataFormat(FrameType type, FrameDataFormat &format);
    };

    class DepthData : public AbstractData
    {
    private:
        std::unique_ptr<float[]> amplitudeData;
        std::unique_ptr<float[]> depthData;
        FrameDataFormat m_depthFormat;
        FrameDataFormat m_amplitudeFormat;

    public:
        DepthData(FrameDataFormat depthFormat, FrameDataFormat amplitudeFormat);
        void *getData(FrameType type);
        int getFrameDataFormat(FrameType type, FrameDataFormat &format);
    };

    class RawData : public AbstractData
    {
    private:
        std::unique_ptr<int16_t[]> rawData;
        FrameDataFormat m_dataFormat;

    public:
        RawData(FrameDataFormat dataFormat);
        void *getData(FrameType type);
        int getFrameDataFormat(FrameType type, FrameDataFormat &format);
    };

    class NodeData
    {
    public:
        NodeData *prev;
        NodeData *next;
    private:
        AbstractData *data;

    public:
        NodeData(FrameDataFormat dataFormat, NodeData *a_pre, NodeData *a_next);
        NodeData(FrameDataFormat depthFormat, FrameDataFormat amplitudeFormat, NodeData *a_pre, NodeData *a_next);
        ~NodeData();
        void *getData(FrameType type);
        int getFrameDataFormat(FrameType type, FrameDataFormat &format);
    };

    class LinkList
    {
    private:
        NodeData *tail;
        NodeData *head;
        NodeData *offset;
        std::vector<NodeData *> vec;
        std::mutex mutex_;

    public:
        LinkList(FrameDataFormat dataFormat);
        LinkList(FrameDataFormat dataFormat, int size);
        LinkList(FrameDataFormat depthFormat, FrameDataFormat amplitudeFormat);
        LinkList(FrameDataFormat depthFormat, FrameDataFormat amplitudeFormat, int size);
        ~LinkList();
        // 压入栈底
        int push2Head(NodeData **node);
        // 出栈
        int popOffset(NodeData **node);
        // 出队
        int DeQueue(NodeData **node);
        // 入队
        int EnQueue(NodeData **node);

    private:
        bool is_element_in_vector(std::vector<NodeData *> v, NodeData *element);
    };
} // namespace ArduCam
#endif
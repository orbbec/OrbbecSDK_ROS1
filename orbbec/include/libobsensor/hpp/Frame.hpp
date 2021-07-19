/**
 * @file Frame.hpp
 * @brief 帧相关类型，主要用于获取帧数据及帧的信息
 * 
 */
#pragma once

#include "libobsensor/hpp/Types.hpp"
#include <memory>
#include <vector>

struct FrameImpl;
struct FrameSetImpl;

namespace ob
{
    class StreamProfile;
    class Filter;

    class OB_EXTENSION_API Frame
    {
    protected:
        std::unique_ptr<FrameImpl> impl_;
    public:
        Frame(std::unique_ptr<FrameImpl> impl);
        virtual ~Frame();
        /**
         * @brief 获取帧的序号
         * 
         * @return uint64_t 返回帧的序号
         */
        uint64_t index();
        /**
         * @brief 获取帧的宽
         * 
         * @return uint32_t 返回帧的宽
         */
        uint32_t width();
        /**
         * @brief 获取帧的高
         * 
         * @return uint32_t 返回帧的高
         */
        uint32_t height();
        /**
         * @brief 获取帧的格式
         * 
         * @return OBFormat 返回帧的格式
         */
        OBFormat format();
        /**
         * @brief 获取帧的类型
         * 
         * @return OBFrameType 返回帧的类型
         */
        OBFrameType type();
        /**
         * @brief 获取帧的硬件时间戳
         * 
         * @return uint64_t 返回帧硬件的时间戳
         */
        uint64_t timeStamp();
        /**
         * @brief 获取帧的系统时间戳
         * 
         * @return uint64_t 返回帧的系统时间戳
         */
        uint64_t systemTimeStamp();
        /**
         * @brief 获取帧的同步时间戳
         * 
         * @return uint64_t 返回帧的系统时间戳
         */
        uint64_t syncTimeStamp();
        /**
         * @brief 获取帧数据
         * 
         * @return void* 返回帧数据
         */
        void* data();
        /**
         * @brief 获取帧数据大小
         * 
         * @return uint32_t 返回帧数据大小
         */
        uint32_t dataSize();
        /**
         * @brief 获取流的类型
         * 
         * @return OBStreamType 返回流的类型
         */
        OBStreamType getStreamType();
        /**
         * @brief 应用过滤器
         * 
         * @param filter 要使用的过滤器
         */
        //void applyFilter(std::shared_ptr<Filter> filter);
    };

    class OB_EXTENSION_API ImuFrame : public Frame {
    public:
        /**
         * @brief 获取当前温度
         *
         * @return float 返回当前温度
         */
        float temperature();

        /**
         * @brief 获取加速度数据
         *
         * @return float3_t 返回加速度数据
         */
        float3_t accelData();

        /**
         * @brief 获取陀螺仪数据
         *
         * @return float3_t 返回陀螺仪数据
         */
        float3_t gyroData();

        /**
         * @brief 获取加速度数据时间戳
         *
         * @return uint64_t 返回加速度数据时间戳
         */
        uint64_t timeStamp();

        /**
         * @brief 获取陀螺仪数据时间戳
         *
         * @return uint64_t 返回陀螺仪数据时间戳
         */
        //uint64_t gyroTimeStamp();

        /**
         * @brief 获取帧的序号
         *
         * @return uint64_t 返回帧的序号
         */
        uint64_t index();

        /**
         * @brief 获取帧的类型
         *
         * @return OBFrameType 返回帧的类型
         */
        OBFrameType type();

        /**
         * @brief 获取流的类型
         *
         * @return OBStreamType 返回流的类型
         */
        OBStreamType streamType();
    };

    class OB_EXTENSION_API FrameSet
    {
    protected:
        std::unique_ptr<FrameSetImpl> impl_;
    public:
        FrameSet();
        FrameSet(std::unique_ptr<FrameSetImpl> impl);
        ~FrameSet();
        /**
         * @brief 帧集合中包含的帧数量
         * 
         * @return uint32_t 返回帧的数量
         */
        uint32_t frameCount();
        /**
         * @brief 获取深度帧
         * 
         * @return std::shared_ptr<Frame> 返回深度帧
         */
        std::shared_ptr<Frame> depthFrame();
        /**
         * @brief 获取彩色帧
         * 
         * @return std::shared_ptr<Frame> 返回彩色帧
         */
        std::shared_ptr<Frame> colorFrame();
        /**
         * @brief 获取红外帧
         * 
         * @return std::shared_ptr<Frame> 返回红外帧
         */
        std::shared_ptr<Frame> infraredFrame();
        /**
         * @brief 获取辅助数据流
         *
         * @return std::shared_ptr<Frame> 返回辅助数据帧
         */
        std::shared_ptr< Frame > assistFrame(); 
        /**
         * @brief 获取点云帧
         *
         * @return  std::shared_ptr<PointsFrame> 返回点云据帧
         */
        std::shared_ptr< Frame > pointsFrame();


        /**
         * @brief 通过传感器类型获取帧
         * 
         * @param frameType 传感器的类型
         * @return std::shared_ptr<Frame> 返回相应类型的帧
         */
        //std::shared_ptr<Frame> getFrame(OBSensorType sensorType);

        friend class Pipeline;
        friend class Filter; 
    };
} // namespace ob

/**
 * @file Sensor.hpp
 * @brief 传感器相关类型，用于获取流配置，开关流，设置及获取传感器属性等操作
 * 
 */
#pragma once

#include "libobsensor/hpp/Types.hpp"
#include <memory>
#include <string>
#include <vector>
#include <functional>

struct SensorImpl;

namespace ob
{
    class StreamProfile;
    class ImuStreamProfile;
    class Device;
    class Frame;
    class ImuFrame;

    using FrameCallback = std::function<void(std::shared_ptr<Frame> frame)>;

    class OB_EXTENSION_API Sensor
    {
    protected:
        std::unique_ptr< SensorImpl > impl_;
        FrameCallback                 callback_;
    public:
        Sensor(std::unique_ptr<SensorImpl> impl);
        virtual ~Sensor();
        /**
         * @brief 传感器类型
         * 
         * @return OBSensorType 返回传感器类型
         */
        OBSensorType type();

        /**
         * @brief 设置结构体定义数据类型的设备属性
         * 
         * @param propertyId 属性id
         * @param data 要设置的属性数据
         * @param dataSize 要设置的属性大小
         */
        void setStructuredData(OBGlobalUnifiedProperty propertyId, const void* data, uint32_t dataSize);
        /**
         * @brief 获取结构体定义数据类型的设备属性
         * 
         * @param propertyId 属性id
         * @param data 获取的属性数据
         * @param dataSize 获取的属性大小
         */
        void getStructuredData(OBGlobalUnifiedProperty propertyId, void* data, uint32_t* dataSize);

        /**
         * @brief 获取raw data类型的传感器属性
         * 
         * @param propertyId 属性id
         * @param callback 获取的数据及进度回调
         * @param async    是否异步执行
         * @return 接口调用是否成功
         */
        bool getRawData( OBGlobalUnifiedProperty propertyId, GetDataCallback callback, bool async=false);
        /**
         * @brief 设置raw data类型的传感器属性
         * 
         * @param propertyId 属性id
         * @param data 要设置的属性数据
         * @param dataSize 设置的属性大小
         * @param callback 进度回调
         * @param async    是否异步执行
         * @return 接口调用是否成功
         */
        bool setRawData( OBGlobalUnifiedProperty propertyId, void *data, uint32_t dataSize, SetDataCallback callback, bool async=false);
        /**
         * @brief 设置int类型的设备属性
         *
         * @param propertyId 属性id
         * @param property 要设置的属性
         */
        void setIntProperty(OBGlobalUnifiedProperty propertyId, int32_t property);

        /**
         * @brief 设置float类型的设备属性
         *
         * @param propertyId 属性id
         * @param property 要设置的属性
         */
        void setFloatProperty(OBGlobalUnifiedProperty propertyId, float property);

        /**
         * @brief 设置bool类型的设备属性
         *
         * @param propertyId 属性id
         * @param property 要设置的属性
         */
        void setBoolProperty(OBGlobalUnifiedProperty propertyId, bool property);

        /**
         * @brief 获取int类型的设备属性
         *
         * @param propertyId 属性id
         * @return int32_t 获取的属性数据
         */
        int32_t getIntProperty(OBGlobalUnifiedProperty propertyId);

        /**
         * @brief 获取float类型的设备属性
         *
         * @param propertyId 属性id
         * @return float 获取的属性数据
         */
        float getFloatProperty(OBGlobalUnifiedProperty propertyId);

        /**
         * @brief 获取bool类型的设备属性
         *
         * @param propertyId 属性id
         * @return bool 获取的属性数据
         */
        bool getBoolProperty(OBGlobalUnifiedProperty propertyId);

        /**
         * @brief 获取int类型的设备属性的范围
         *
         * @param propertyId 属性id
         * @return OBIntPropertyRange 属性的范围
         */
        OBIntPropertyRange getIntPropertyRange(OBGlobalUnifiedProperty propertyId);

        /**
         * @brief 获取float类型的设备属性的范围
         *
         * @param propertyId 属性id
         * @return OBFloatPropertyRange 属性的范围
         */
        OBFloatPropertyRange getFloatPropertyRange(OBGlobalUnifiedProperty propertyId);

         /**
         * @brief 获取Bool类型的设备属性的范围
         *
         * @param propertyId 属性id
         * @return OBBoolPropertyRange 属性的范围
         */
        OBBoolPropertyRange getBoolPropertyRange(OBGlobalUnifiedProperty propertyId);

        /**
         * @brief 判断传感器属性是否支持
         * 
         * @param propertyId 属性id
         * @return true 支持该属性
         * @return false 不支持该属性
         */
        bool isPropertySupported(OBGlobalUnifiedProperty propertyId);
        /**
         * @brief 获取传感器的流配置
         * 
         * @return std::vector<std::shared_ptr<StreamProfile>> 返回流配置列表
         */
        std::vector<std::shared_ptr<StreamProfile>> getStreamProfiles();
        
        /**
         * @brief 开启流并设置帧数据回调
         * 
         * @param streamProfile 流的配置
         * @param callback 设置帧数据到达时的回调
         */
        void start(std::shared_ptr<StreamProfile> streamProfile, FrameCallback callback);
        /**
         * @brief 停止流
         * 
         */
        void stop();

        /**
         * @brief 切换流的配置
         *
         * @param streamProfile 流的配置
         */
        void switchProfile(std::shared_ptr<StreamProfile> streamProfile);
      
    };

    using ImuFrameCallback = std::function<void(std::shared_ptr<ImuFrame> imuFrame)>;

    class OB_EXTENSION_API ImuSensor : protected Sensor {
        public:
          /**
         * @brief 获取传感器的流配置
         *
         * @return std::vector<std::shared_ptr<StreamProfile>> 返回流配置列表
         */
        std::vector< std::shared_ptr< ImuStreamProfile > > getImuStreamProfiles();
        /**
         * @brief 读取默认参数
         *
         * @return CalibrationExtrinsics 返回默认参数
         */
        CalibrationExtrinsics imuGetExtrinsics();

        /**
         * @brief 开启imu流
         * @param callback 设置imu帧到达时的回调
         */
        void imuStart(std::shared_ptr<ImuStreamProfile> imuStreamProfile,ImuFrameCallback callback);

        /**
         * @brief 关闭imu流
         */
        void imuStop();

      
    };

} // namespace ob

/**
 * @file ImuSensor.hpp
 * @brief IMU传感器相关类型，用于开关流，设置及获取传感器属性等操作
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
    class Device;
    class ImuFrame;

    using ImuFrameCallback = std::function<void(std::shared_ptr<ImuFrame> imuFrame)>;

    class OB_EXTENSION_API ImuSensor
    {
    private:
        std::unique_ptr<SensorImpl> impl_;
        ImuFrameCallback callback_;
    public:
        ImuSensor(std::unique_ptr<SensorImpl> impl);
        virtual ~ImuSensor();
        /**
         * @brief 传感器类型
         * 
         * @return OBSensorType 返回传感器类型
         */
        OBSensorType type();

        /**
         * @brief 设置raw data类型的设备属性
         * 
         * @param propertyId 属性id
         * @param data 要设置的属性数据
         * @param dataSize 要设置的属性大小
         */
        void setProperty(OBGlobalUnifiedProperty propertyId, const void* data, int dataSize);
        /**
         * @brief 获取raw data类型的设备属性
         * 
         * @param propertyId 属性id
         * @param data 获取的属性数据
         * @param dataSize 获取的属性大小
         */
        void getProperty(OBGlobalUnifiedProperty propertyId, void* data, int* dataSize);

        /**
         * @brief 获取raw data类型的传感器属性
         * 
         * @param propertyId 属性id
         * @param data 获取的属性数据
         */
        void getRawData(OBGlobalUnifiedProperty propertyId, RawData* data);
        /**
         * @brief 设置raw data类型的传感器属性
         * 
         * @param propertyId 属性id
         * @param data 要设置的属性数据
         */
        void setRawData(OBGlobalUnifiedProperty propertyId, const RawData* data);
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
         * @brief 判断传感器属性是否支持
         * 
         * @param propertyId 属性id
         * @return true 支持该属性
         * @return false 不支持该属性
         */
        bool isPropertySupported(OBGlobalUnifiedProperty propertyId);

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
        void imuStart(ImuFrameCallback callback);

        /**
         * @brief 关闭imu流
         */
        void imuStop();
    };
} // namespace ob

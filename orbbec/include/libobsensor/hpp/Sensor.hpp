/**
 * @file Sensor.hpp
 * @brief 传感器相关类型，用于获取流配置，开关流，设置及获取传感器属性等操作
 *
 */
#pragma once

#include "Types.hpp"

#include <functional>
#include <memory>

struct SensorImpl;
struct SensorListImpl;

namespace ob {
class StreamProfile;
class StreamProfileList;
class Device;
class Frame;

using FrameCallback = std::function< void( std::shared_ptr< Frame > frame ) >;

class OB_EXTENSION_API Sensor {
protected:
    std::unique_ptr< SensorImpl > impl_;
    FrameCallback                 callback_;

public:
    Sensor( std::unique_ptr< SensorImpl > impl );
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
    void setStructuredData( OBGlobalUnifiedProperty propertyId, const void* data, uint32_t dataSize );
    /**
     * @brief 获取结构体定义数据类型的设备属性
     *
     * @param propertyId 属性id
     * @param data 获取的属性数据
     * @param dataSize 获取的属性大小
     */
    void getStructuredData( OBGlobalUnifiedProperty propertyId, void* data, uint32_t* dataSize );

    /**
     * @brief 获取raw data类型的传感器属性
     *
     * @param propertyId 属性id
     * @param callback 获取的数据及进度回调
     * @param async    是否异步执行
     * @return 接口调用是否成功
     */
    bool getRawData( OBGlobalUnifiedProperty propertyId, GetDataCallback callback, bool async = false );
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
    bool setRawData( OBGlobalUnifiedProperty propertyId, void* data, uint32_t dataSize, SetDataCallback callback, bool async = false );
    /**
     * @brief 设置int类型的设备属性
     *
     * @param propertyId 属性id
     * @param property 要设置的属性
     */
    void setIntProperty( OBGlobalUnifiedProperty propertyId, int32_t property );

    /**
     * @brief 设置float类型的设备属性
     *
     * @param propertyId 属性id
     * @param property 要设置的属性
     */
    void setFloatProperty( OBGlobalUnifiedProperty propertyId, float property );

    /**
     * @brief 设置bool类型的设备属性
     *
     * @param propertyId 属性id
     * @param property 要设置的属性
     */
    void setBoolProperty( OBGlobalUnifiedProperty propertyId, bool property );

    /**
     * @brief 获取int类型的设备属性
     *
     * @param propertyId 属性id
     * @return int32_t 获取的属性数据
     */
    int32_t getIntProperty( OBGlobalUnifiedProperty propertyId );

    /**
     * @brief 获取float类型的设备属性
     *
     * @param propertyId 属性id
     * @return float 获取的属性数据
     */
    float getFloatProperty( OBGlobalUnifiedProperty propertyId );

    /**
     * @brief 获取bool类型的设备属性
     *
     * @param propertyId 属性id
     * @return bool 获取的属性数据
     */
    bool getBoolProperty( OBGlobalUnifiedProperty propertyId );

    /**
     * @brief 获取int类型的设备属性的范围
     *
     * @param propertyId 属性id
     * @return OBIntPropertyRange 属性的范围
     */
    OBIntPropertyRange getIntPropertyRange( OBGlobalUnifiedProperty propertyId );

    /**
     * @brief 获取float类型的设备属性的范围
     *
     * @param propertyId 属性id
     * @return OBFloatPropertyRange 属性的范围
     */
    OBFloatPropertyRange getFloatPropertyRange( OBGlobalUnifiedProperty propertyId );

    /**
     * @brief 获取Bool类型的设备属性的范围
     *
     * @param propertyId 属性id
     * @return OBBoolPropertyRange 属性的范围
     */
    OBBoolPropertyRange getBoolPropertyRange( OBGlobalUnifiedProperty propertyId );

    /**
     * @brief 判断传感器属性是否支持
     *
     * @param propertyId 属性id
     * @return true 支持该属性
     * @return false 不支持该属性
     */
    bool isPropertySupported( OBGlobalUnifiedProperty propertyId );

    /**
     * @brief 获取传感器的流配置列表
     *
     * @return std::shared_ptr<StreamProfileList> 返回流配置列表
     */
    const std::shared_ptr< StreamProfileList > getStreamProfileList();

    /**
     * @brief 开启流并设置帧数据回调
     *
     * @param streamProfile 流的配置
     * @param callback 设置帧数据到达时的回调
     */
    void start( std::shared_ptr< StreamProfile > streamProfile, FrameCallback callback );
    /**
     * @brief 停止流
     *
     */
    void stop();
};
class OB_EXTENSION_API SensorList {
private:
    std::unique_ptr< SensorListImpl > impl_;

public:
    SensorList( std::unique_ptr< SensorListImpl > impl );

    virtual ~SensorList();

    /**
     * @brief 获取Sensor数量
     *
     * @return uint32_t 返回Sensor的数量
     */
    uint32_t count();

    /**
     * @brief 获取指定Sensor的类型
     *
     * @param index Sensor索引
     * @return OBSensorType 返回Sensor类型
     */
    OBSensorType type( uint32_t index );

    /**
     * @brief 通过索引号获取Sensor
     *
     * @param index 要创建设备的索，范围 [0, count-1]，如果index超出范围将抛异常
     * @return std::shared_ptr<Sensor> 返回Sensor对象
     */
    std::shared_ptr< Sensor > getSensor( uint32_t index );

    /**
     * @brief 通过Sensor类型获取Sensor
     *
     * @param sensorType 要获取的Sensor类型
     * @return std::shared_ptr<Sensor> 返回Sensor对象，如果指定类型Sensor不存在，将返回空
     */
    std::shared_ptr< Sensor > getSensor( OBSensorType sensorType );
};

}  // namespace ob

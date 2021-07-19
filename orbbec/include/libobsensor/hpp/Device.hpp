/**
 * @file Device.hpp
 * @brief 设备相关类型，包括获取创建设备，设置及获取设备属性，获取传感器等操作
 * 
 */
#pragma once
#include "libobsensor/hpp/Types.hpp"
#include <memory>
#include <string>
#include <vector>

struct DeviceImpl;
struct DeviceInfoImpl;
struct DeviceListImpl;

namespace ob
{
    class Sensor;
    class StreamProfile;
    class Context;
    class DeviceInfo;

    class OB_EXTENSION_API Device
    {
    private:
        std::unique_ptr<DeviceImpl> impl_;
    public:
        Device(std::unique_ptr<DeviceImpl> impl);
        virtual ~Device();
        /**
         * @brief 获取设备信息
         * 
         * @return std::shared_ptr<DeviceInfo> 返回设备的信息
         */
        std::shared_ptr<DeviceInfo> getDeviceInfo();
        /**
         * @brief 获取设备的所有传感器
         * 
         * @return std::vector<std::shared_ptr<Sensor>> 返回传感器列表
         */
        std::vector<std::shared_ptr<Sensor>> getSensors();
        /**
         * @brief 获取设备的传感器
         * 
         * @param sensorType 传感器类型
         * @return std::shared_ptr<Sensor> 返回传感器对象
         */
        std::shared_ptr<Sensor> getSensor(OBSensorType sensorType);
        /**
         * @brief 获取传感器的流配置
         * 
         * @param sensorType 传感器类型
         * @return std::vector<std::shared_ptr<StreamProfile>> 流配置的列表
         */
        std::vector<std::shared_ptr<StreamProfile>> getStreamProfiles(OBSensorType sensorType);

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
         * @brief 获取int类型的设备属性的范围(包括当前值和默认值)
         *
         * @param propertyId 属性id
         * @return OBIntPropertyRange 属性的范围
         */
        OBIntPropertyRange getIntPropertyRange(OBGlobalUnifiedProperty propertyId);

        /**
         * @brief 获取float类型的设备属性的范围(包括当前值和默认值)
         *
         * @param propertyId 属性id
         * @return OBFloatPropertyRange 属性的范围
         */
        OBFloatPropertyRange getFloatPropertyRange(OBGlobalUnifiedProperty propertyId);

         /**
         * @brief 获取bool类型的设备属性的范围(包括当前值和默认值)
         *
         * @param propertyId 属性id
         * @return OBBoolPropertyRange 属性的范围
         */
        OBBoolPropertyRange getBoolPropertyRange(OBGlobalUnifiedProperty propertyId);

        /**
         * @brief AHB写寄存器
         *
         * @param reg 要写入的寄存器
         * @param mask 要写入的掩码
         * @param value 要写入的值
         */
        void writeAHB(uint32_t reg, uint32_t mask, uint32_t value);

        /**
         * @brief AHB读寄存器
         *
         * @param reg 要读取的寄存器
         * @param mask 要读取的掩码
         * @param value 读取的值返回
         */
        void readAHB( uint32_t reg, uint32_t mask, uint32_t* value );

         /**
         * @brief I2C写寄存器
         *
         * @param reg 要写入的I2C模块ID
         * @param reg 要写入的寄存器
         * @param mask 要写入的掩码
         * @param value 要写入的值
         */
        void writeI2C(uint32_t moduleId, uint32_t reg, uint32_t mask, uint32_t value);

        /**
         * @brief I2C读寄存器
         *
         * @param reg 要读取的I2C模块ID
         * @param reg 要读取的寄存器
         * @param mask 要读取的掩码
         * @param value 读取的值返回
         */
        void readI2C(uint32_t moduleId, uint32_t reg, uint32_t mask, uint32_t* value );

        /**
         * @brief 设置写入Flash的属性
         *
         * @param offset flash 偏移地址
         * @param data 要写入的属性数据
         * @param dataSize 要写入的属性大小
         * @param callback 写flash进度回调
         * @param async    是否异步执行
         * @return 接口调用是否成功
         */
        bool writeFlash(uint32_t offset, const void *data, uint32_t dataSize, SetDataCallback callback, bool async=false);

        /**
         * @brief 读取Flash的属性
         *
         * @param offset flash 偏移地址
         * @param data 读取的属性数据
         * @param dataSize 获取的属性大小
         * @param callback 读flash返回的数据及进度回调
         * @param async    是否异步执行
         * @return 接口调用是否成功
         */
        bool readFlash(uint32_t offset, uint32_t dataSize, GetDataCallback callback, bool async=false);

        /**
         * @brief 设置raw data类型的设备属性数据[异步回调]
         * 
         * @param propertyId 属性id
         * @param data 要设置的属性数据
         * @param dataSize 要设置的属性数据大小
         * @param callback rawdata设置进度回调
         * @param async    是否异步执行
         * @return 接口调用是否成功
         */
        bool setRawData(OBGlobalUnifiedProperty propertyId,  const void *data, uint32_t dataSize, SetDataCallback callback, bool async=false);
       
        /**
         * @brief 获取raw data类型的设备属性数据[异步回调]
         * 
         * @param propertyId 属性id
         * @param data 获取的属性数据
         * @param dataSize 获取的属性大小
         * @param callback 获取返回的数据及进度回调
         * @param async    是否异步执行
         * @return 接口调用是否成功
         */
        bool getRawData(OBGlobalUnifiedProperty propertyId, GetDataCallback callback, bool async=false);

        /**
         * @brief 设置raw data类型的设备属性
         *
         * @param propertyId 属性id
         * @param data 要设置的属性数据
         * @param dataSize 要设置的属性大小
         */
        void setStructuredData( OBGlobalUnifiedProperty propertyId, const void *data, uint32_t dataSize );
        
        /**
         * @brief 获取raw data类型的设备属性
         *
         * @param propertyId 属性id
         * @param data 获取的属性数据
         * @param dataSize 获取的属性大小
         */
        void getStructuredData( OBGlobalUnifiedProperty propertyId, void *data, uint32_t* dataSize );

        /**
         * @brief 判断设备的属性是否支持
         * 
         * @param propertyId 属性id
         * @return true 支持该属性
         * @return false 不支持该属性
         */
        bool isPropertySupported(OBGlobalUnifiedProperty propertyId);

        /**
         * 
         * @brief 更新设备时间戳（向设备授时，同步本地系统时间到设备）
         * @return uint64_t 命令往返时间延时（round trip time， rtt）
         */
        uint64_t updateDeviceTime();

        /**
         * @brief 升级设备固件
         * 
         * @param filePath 固件的路径
         * @param callback 固件升级进度及状态回调
         * @param async    是否异步执行
         * @return 接口调用是否成功
         */
        bool deviceUpgrade( const char *filePath, DeviceUpgradeCallback callback, bool async=true);
        /**
         * @brief 重新设置设备的所有属性
         * 
         */
        void deviceReset();
        /**
         * @brief 发送文件到设备端指定路径[异步回调]
         *
         * @param filePath 原文件路径
         * @param dstPath 设备端接受保存路径
         * @param callback 文件传输回调 
         * @param async    是否异步执行
         * @return 接口调用是否成功
         */
        bool sendFile( const char *filePath , const char * dstPath, SendFileCallback callback, bool async=true);

        /**
         * @brief 设置设备状态改变回调函数
         *
         * @param callback 设备状态改变（如，由于温度过高自动降低帧率或关流等）时触发的回调函数
         */
        void setDeviceStateChangedCallback(DeviceStateChangedCallback callback);

        /**
         * @brief 验证设备授权码
         * @param authCode 授权码
         * @return bool 激活是否成功
         */
        bool activateAuthorization(const char * authCode);

        /**
         * @brief 写入设备授权码
         * @param authCode 授权码
         */
        void writeAuthorizationCode(const char * authCodeStr);

        /**
         * @brief 获取当前内参（会根据sensor镜像状态做转换）
         * @param sensorType 需要获取内参的Sensor
         * 
         * @return OBCameraIntrinsic 内参结构体
         */
        OBCameraIntrinsic getCameraIntrinsic(OBSensorType sensorType);

        /**
         * @brief 获取当前去畸变参数（会根据sensor镜像状态做转换）
         * @param sensorType 需要获取内参的Sensor
         * 
         * @return OBCameraDistortion 去畸变参数结构体
         */
        OBCameraDistortion getCameraDistortion(OBSensorType sensorType);

        /**
         * @brief 获取旋转矩阵（会根据sensor镜像状态做转换）
         * 
         * @return Transform 旋转矩阵结构体
         */
        OBD2CTransform getD2CTransform();
        

        friend class Pipeline;
    };

    class OB_EXTENSION_API DeviceInfo
    {
    private:
        std::unique_ptr<DeviceInfoImpl> impl_;
    public:
        DeviceInfo(std::unique_ptr<DeviceInfoImpl> impl);
        virtual ~DeviceInfo();
        /**
         * @brief 获取设备名称
         * 
         * @return const char * 返回设备名称
         */
        const char *name();
        /**
         * @brief 获取设备的pid
         * 
         * @return int 返回设备的pid
         */
        int pid();
        /**
         * @brief 获取设备的vid
         * 
         * @return int 返回设备的vid
         */
        int vid();
        /**
         * @brief 获取设备的uid
         * 
         * @return const char * 返回设备的uid
         */
        const char * uid();
        /**
         * @brief 获取设备的序列号
         * 
         * @return const char * 返回设备的序列号
         */
        const char * serialNumber();
        /**
         * @brief 获取固件的版本号
         * 
         * @return int 返回固件的版本号
         */
        const char * firmwareVersion();

        /**
         * @brief 获取usb连接类型
         * 
         * @return std::string 返回usb连接类型
         */
        std::string usbType();

        friend class Context;
        friend class DeviceList;
    };

    class OB_EXTENSION_API DeviceList
    {
    private:
        std::unique_ptr<DeviceListImpl> impl_;
    public:
        DeviceList(std::unique_ptr<DeviceListImpl> impl);
        virtual ~DeviceList();

        /**
         * @brief 获取设备信息
         * 
         * @param index 设备的索引
         * @return std::shared_ptr<DeviceInfo> 返回设备信息
         */
        //std::shared_ptr<DeviceInfo> getDeviceInfo(int index);
        /**
         * @brief 获取设备数量
         * 
         * @return uint32_t 返回设备的数量
         */
        uint32_t deviceCount();

        /**
         * @brief 获取指定设备的名称
         *
         * @param index 设备索引
         * @return int 返回设备的名称
         */
        const char * name(uint32_t index);
        /**
         * @brief 获取指定设备的pid
         *
         * @param index 设备索引
         * @return int 返回设备的pid
         */
        int pid(uint32_t index);
        /**
         * @brief 获取指定设备的vid
         *
         * @param index 设备索引
         * @return int 返回设备的vid
         */
        int vid(uint32_t index);
        /**
         * @brief 获取指定设备的uid
         *
         * @param index 设备索引
         * @return const char * 返回设备的uid
         */
        const char * uid(uint32_t index);

        /**
         * @brief 创建设备对象
         * 
         * @param deviceInfo 设备信息
         * @return std::shared_ptr<Device> 返回设备对象
         */
        //std::shared_ptr<Device> createDevice(std::shared_ptr<DeviceInfo> deviceInfo);
        /**
         * @brief 创建设备对象
         * 
         * @param index 要创建设备的索引
         * @return std::shared_ptr<Device> 返回设备对象
         */
        std::shared_ptr<Device> createDevice(uint32_t index);
    };
} // namespace ob

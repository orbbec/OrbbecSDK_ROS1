/**
 * @file Context.hpp
 * @brief SDK上下文环境类，底层SDK的入口，用于获取设备列表，处理设备的回调，日志等级的设置操作
 * 
 */
#pragma once

#include "libobsensor/hpp/Types.hpp"

#include <functional>
#include <memory>
#include <vector>

struct ContextImpl;

namespace ob {
class Device;
class DeviceInfo;
class DeviceList;

class OB_EXTENSION_API Context {
private:
    std::unique_ptr< ContextImpl > impl_;

public:
    Context();
    virtual ~Context();
    
    /**
     * @brief 获取设备列表
     * 
     * @return std::shared_ptr<DeviceList> 
     */
    std::shared_ptr<DeviceList> queryDeviceList();

    using DeviceChangedCallback = std::function< void( std::shared_ptr< DeviceList > removedList, std::shared_ptr< DeviceList > addedList ) >;

    /**
     * @brief 设置设备插拔回调函数
     * 
     * @param callback 设备插拔时触发的回调函数
     */
    void setDeviceChangedCallback( DeviceChangedCallback callback );

    /**
     * @brief 设置日志的输出等级
     * 
     * @param log log的输出等级
     */
    void setLoggerServerity( OBLogServerity log );

    /**
     * @brief 设置日志输出到文件
     * 
     * @param log 日志的输出等级
     * @param fileName 输出的文件名
     */
    void setLoggerToFile( OBLogServerity log, const char* fileName );

    /**
     * @brief 设置日志输出到终端
     * 
     * @param log 日志的输出等级
     */
    void setLoggerToConsole( OBLogServerity log );
};
}  // namespace ob

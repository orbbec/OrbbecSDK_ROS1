/**
 * @file Context.hpp
 * @brief SDK上下文环境类，底层SDK的入口，用于获取设备列表，处理设备的回调，日志等级的设置操作
 *
 */
#pragma once

#include "Types.hpp"

#include <functional>
#include <memory>

struct ContextImpl;

namespace ob {
class Device;
class DeviceInfo;
class DeviceList;

class OB_EXTENSION_API Context {
private:
    std::unique_ptr< ContextImpl > impl_;

public:
    /**
     * @brief context是描述SDK的runtime一个管理类，负责SDK的资源申请与释放
     * context具备多设备的管理能力，负责枚举设备，监听设备回调，启用多设备同步等功能
     *
     */
    Context();
    virtual ~Context();

    /**
     * @brief 获取枚举到设备列表
     *
     * @return std::shared_ptr<DeviceList>返回设备列表类的指针
     */
    std::shared_ptr< DeviceList > queryDeviceList();

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

    /**
     * @brief 启动多设备同步功能，同步已创建设备的时钟(需要使用的设备支持该功能)
     *
     * @param repeatInterval 定时同步时间间隔（单位ms；如果repeatInterval=0，表示只同步一次，不再定时执行）
     */
    void enableMultiDeviceSync( uint64_t repeatInterval );
};
}  // namespace ob

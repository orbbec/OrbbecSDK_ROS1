/**
 * @file Pipeline.hpp
 * @brief SDK的高级API类型，可以快速实现开关流，帧同步，软件滤波，生成点云等操作
 *
 */
#pragma once

#include "Types.hpp"

#include <functional>
#include <memory>

struct PipelineImpl;
struct ConfigImpl;

namespace ob {
class FrameSet;
class Frame;
class Device;
class Config;
class StreamProfile;
class TestFilter;
class StreamProfileList;

class OB_EXTENSION_API Pipeline {
private:
    std::unique_ptr< PipelineImpl > impl_;

public:
    /**
     * @brief Pipeline 是SDK的高级接口，适用于应用，算法等重点关注RGBD数据流常见，Pipeline在SDK内部可以提供对齐，同步后的FrameSet桢集合
     * 直接方便客户使用。
     * Pipeline()无参数版本，默认打开连接到OS的设备列表中的第一个设备。若应用已经通过DeviceList获取设备，此时打开Pipeline()会抛出设备已经创建异常。
     * 需要开发者捕获异常处理。
     */
    Pipeline();
    /**
     * @brief
     * Pipeline(std::shared_ptr< Device > device )函数，适用于多设备操作常见，此时需要通过DeviceList获取多个设备，通过该接口实现device和pipeline绑定。
     */
    Pipeline( std::shared_ptr< Device > device );
    ~Pipeline();
    /**
     * @brief 启动pipeline并配置参数
     *
     * @param config pipeline的参数配置
     */
    void start( std::shared_ptr< Config > config );

    using FrameSetCallback = std::function< void( std::shared_ptr< FrameSet > frame ) >;
    /**
     * @brief 启动pipeline并设置帧集合数据回调
     *
     * @param config pipeline的参数配置
     * @param callback 设置帧集合中的所有帧数据都到达时触发回调
     */
    void start( std::shared_ptr< Config > config, FrameSetCallback callback );
    /**
     * @brief 停止pipeline
     *
     */
    void stop();
    /**
     * @brief 获取pipeline的配置参数
     *
     * @return std::shared_ptr<Config> 返回配置的参数
     */
    std::shared_ptr< Config > getConfig();
    /**
     * @brief 等待帧集合数据
     *
     * @param timeout_ms 等待超时时间(毫秒)
     * @return std::shared_ptr<FrameSet> 返回等待的帧集合数据
     */
    std::shared_ptr< FrameSet > waitForFrames( uint32_t timeout_ms );
    /**
     * @brief 获取设备对象
     *
     * @return std::shared_ptr<Device> 返回设备对象
     */
    std::shared_ptr< Device > getDevice();
    /**
     * @brief 获取指定传感器的流配置
     *
     * @param sensorType 传感器的类型
     * @return std::shared_ptr<StreamProfileList> 返回流配置列表
     */
    std::shared_ptr< StreamProfileList > getStreamProfileList( OBSensorType sensorType );
    /**
     * @brief 获取所有传感器的所有流的配置
     *
     * @return  std::shared_ptr<StreamProfileList> 返回流配置列表
     */
    std::shared_ptr< StreamProfileList > getAllStreamProfileList();

    /**
     * @brief 打开帧同步功能
     *
     */
    void enableFrameSync();
    /**
     * @brief 关闭帧同步功能
     *
     */
    void disableFrameSync();

    /**
     * @brief 创建Filter
     *
     * @tparam T Filter类，如果使用不支持的类，函数将会返回nullptr
     *
     * @return std::shared_ptr<T> 返回Filter对象智能指针，
     *
     */
    template < typename T > std::shared_ptr< T > createFilter();
};

class OB_EXTENSION_API Config {
private:
    std::unique_ptr< ConfigImpl > impl_;

public:
    Config();
    ~Config();

    /**
     * @brief 设置要打开的流配置
     *
     * @param streamProfile 流的配置
     */
    void enableStream( std::shared_ptr< StreamProfile > streamProfile );
    /**
     * @brief 设置打开所有的流
     *
     */
    void enableAllStream();
    /**
     * @brief 设置要关闭的流配置
     *
     * @param streamType 流的配置
     */
    void disableStream( OBStreamType streamType );
    /**
     * @brief 设置关闭所有的流
     *
     */
    void disableAllStream();

    friend class Pipeline;
};

typedef std::function< void( std::shared_ptr< Frame > ) > FilterCallback;

class OB_EXTENSION_API Filter {
public:
    virtual ~Filter() = default;
    /**
     * @brief 重置filter
     *
     */
    virtual void reset();

    /**
     * @brief 处理frame（同步接口）
     *
     * @param frameSet 需要处理的frame，处理结果会直接覆盖其本身
     */
    virtual void process( std::shared_ptr< Frame > frame );

    /**
     * @brief 启动处理线程（异步回调接口）
     *
     * @return bool 启动是否成功
     */
    virtual bool start();
    /**
     * @brief 停止处理线程（异步回调接口）
     */
    virtual void stop();
    /**
     * @brief 压入待处理frameset到缓存（异步回调接口）
     *
     * @param frameSet 待处理的frame处理结果通过回调函数返回（由于是对指针进行操作，处理完结果也会覆盖掉原有对象）
     */
    virtual void pushFrame( std::shared_ptr< Frame > frame );
    /**
     * @brief 设置回调函数（异步回调接口）
     *
     * @param callback 处理结果回调
     */
    virtual void setCallBack( FilterCallback callback );

protected:
    friend class Pipeline;
    // 只可通过子类构造
    Filter();
    std::shared_ptr< FilterImpl > impl_;
};
}  // namespace ob

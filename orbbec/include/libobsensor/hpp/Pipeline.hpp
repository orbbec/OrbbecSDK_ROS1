/**
 * @file Pipeline.hpp
 * @brief SDK的高级API类型，可以快速实现开关流，帧同步，软件滤波，生成点云等操作
 * 
 */
#pragma once

#include "libobsensor/hpp/Types.hpp"
#include <memory>
#include <vector>
#include <functional>

struct PipelineImpl;
struct ConfigImpl;

namespace ob
{
    class FrameSet;
    class Device;
    class Config;
    class StreamProfile;
    class TestFilter;

    class OB_EXTENSION_API Pipeline
    {
    private:
        std::unique_ptr<PipelineImpl> impl_;
    public:
        Pipeline();
        Pipeline(std::shared_ptr<Device> device);
        ~Pipeline();
        /**
         * @brief 以默认配置启动pipeline
         * 
         */
        void start();
        /**
         * @brief 启动pipeline并配置参数
         * 
         * @param config pipeline的参数配置
         */
        void start(std::shared_ptr<Config> config);

        using FrameSetCallback = std::function<void(std::shared_ptr<FrameSet> frame)>;
        /**
         * @brief 启动pipeline并设置帧集合数据回调
         * 
         * @param config pipeline的参数配置
         * @param callback 设置帧集合中的所有帧数据都到达时触发回调
         */
        void start(std::shared_ptr<Config> config, FrameSetCallback callback);
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
        std::shared_ptr<Config> getConfig();
        /**
         * @brief 等待帧集合数据
         * 
         * @param timeout_ms 等待超时时间(毫秒)
         * @return std::shared_ptr<FrameSet> 返回等待的帧集合数据
         */
        std::shared_ptr<FrameSet> waitForFrames(uint32_t timeout_ms);
        /**
         * @brief 获取设备对象
         * 
         * @return std::shared_ptr<Device> 返回设备对象
         */
        std::shared_ptr<Device> getDevice();
        /**
         * @brief 获取指定传感器的流配置
         * 
         * @param sensorType 传感器的类型
         * @return std::vector<std::shared_ptr<StreamProfile>> 返回流配置列表
         */
        std::vector<std::shared_ptr<StreamProfile>> getStreamProfiles(OBSensorType sensorType);
        /**
         * @brief 获取所有传感器的所有流的配置
         * 
         * @return std::vector<std::shared_ptr<StreamProfile>> 返回流配置列表
         */
        std::vector<std::shared_ptr<StreamProfile>> getAllStreamProfiles();
        /**
         * @brief 设置数据流的旋转方向
         *
         */
        void setOrientation(OBOrientationType type);
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
         * @brief 打开ROI裁切功能
         *
         *  @param x 左上顶点x坐标
         *  @param y 左上顶点y坐标
         *  @param w 区域宽
         *  @param y 区域高
         */
        void enableRoiRect(uint32_t x,uint32_t y,uint32_t w,uint32_t h);
        /**
         * @brief 关闭ROI裁切功能
         *
         */
        void disableRoiRect();
        /**
         * @brief 设置裁剪的区域
         *
         */
        void setCropRect( uint32_t x, uint32_t y, uint32_t w, uint32_t h );

        void debugSetSyncGap( uint32_t gap, bool debug );


        /**
         * @brief 创建Filter
         * 
         * @tparam T Filter类，如果使用不支持的类，函数将会返回nullptr
         * 
         * @return std::shared_ptr<T> 返回Filter对象智能指针，
         *
         */
        template<typename T> 
        std::shared_ptr<T> createFilter();
    };

    class OB_EXTENSION_API Config
    {
    private:
        std::unique_ptr<ConfigImpl> impl_;
    public:
        Config();
        ~Config();
        /**
         * @brief 设置要打开的复合流主流+辅助流
         *
         * @param streamProfile 流的配置
         */
        void enableMultiStream( std::shared_ptr< StreamProfile > streamProfile );
        /**
         * @brief 设置要打开的流配置
         * 
         * @param streamProfile 流的配置
         */
        void enableStream(std::shared_ptr<StreamProfile> streamProfile);
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
        void disableStream(OBStreamType streamType);
        /**
         * @brief 设置关闭所有的流
         * 
         */
        void disableAllStream();

        friend class Pipeline;
    };

    typedef std::function< void( std::shared_ptr< FrameSet > ) > FilterCallback;

    class OB_EXTENSION_API Filter {
    public:
        virtual ~Filter() = default;
        /**
        * @brief 重置filter
        * 
        */
        virtual void reset();

        /**
        * @brief 处理frameset（同步接口）
        * 
        * @param frameSet 需要处理的frameset，处理结果会直接覆盖其本身
        */
        virtual void process( std::shared_ptr< FrameSet > frameSet );

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
        * @param frameSet 待处理的frameSet，处理结果通过回调函数返回（由于是对指针进行操作，处理完结果也会覆盖掉原有对象）
        */
        virtual void pushFrameSet( std::shared_ptr< FrameSet > frameSet );
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
    class OB_EXTENSION_API D2CFilter : public Filter { 
    public:
        /**
        * @brief 设置相机参数
        * 
        * @param param 相机内外参数
        */
        void setCameraParam(CAMERA_PARA *param);
    private:
        friend class Pipeline; 
         /**
         * @brief 构造函数，必须通过pipeline构造
         *
         */
        D2CFilter(){}; 
    };

    class OB_EXTENSION_API PointCloudFilter : public Filter { 
    public:
        
        /**
        * @brief 设置点云类型参数
        * 
        * @param type 点云类型 深度点云或RGBD点云
        */
        void setCreatePointFormat(OBFormat type);

        /**
        * @brief 设置相机参数
        * 
        * @param param 相机内外参数
        */
        void setCameraPara(CAMERA_PARA param);

        /**
        * @brief  使能对齐模式（D2C模式下需要开启，作为算法选用那组相机内参的依据）
        * 
        * @param status 对齐状态，True：开启对齐； False：关闭对齐
        */
        void enableAlignedMode(bool enable);
        
    private:
        friend class Pipeline; 
         /**
         * @brief 构造函数，必须通过pipeline构造
         *
         */
        PointCloudFilter(){}; 
    };

} // namespace ob

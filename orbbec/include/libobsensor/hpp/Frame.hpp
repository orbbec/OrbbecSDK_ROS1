/**
 * @file Frame.hpp
 * @brief 帧相关类型，主要用于获取帧数据及帧的信息
 *
 */
#pragma once

#include "Types.hpp"

#include <memory>
#include <typeinfo>

/**
 *    Frame类继承关系：
 *                 Frame
 *                   |
 *            +------+---+
 *            |          |
 *      VideoFrame   FrameSet
 *          |
 *       +--+------+---------+
 *      |          |         |
 *  ColorFrame DepthFrame IRFrame
 *
 */

struct FrameImpl;

namespace ob {
class StreamProfile;
class Filter;

class OB_EXTENSION_API Frame : public std::enable_shared_from_this< Frame > {
protected:
    std::unique_ptr< FrameImpl > impl_;

public:
    Frame( std::unique_ptr< FrameImpl > impl );
    Frame( Frame& frame );

    virtual ~Frame();
    /**
     * @brief 获取帧的类型
     *
     * @return OBFrameType 返回帧的类型
     */
    virtual OBFrameType type();

    /**
     * @brief 获取帧的格式
     *
     * @return OBFormat 返回帧的格式
     */
    virtual OBFormat format();

    /**
     * @brief 获取帧的序号
     *
     * @return uint64_t 返回帧的序号
     */
    virtual uint64_t index();

    /**
     * @brief 获取帧数据
     *
     * @return void* 返回帧数据
     */
    virtual void* data();

    /**
     * @brief 获取帧数据大小
     *
     * @return uint32_t 返回帧数据大小
     */
    virtual uint32_t dataSize();

    /**
     * @brief 检查帧对象的运行时类型是否与给定类型兼容
     *
     * @tparam T 给定的类型
     * @return bool 返回结果
     */
    template < typename T > bool is();

    /**
     * @brief 帧对象类型转换
     *
     * @tparam T 目标类型
     * @return std::shared_ptr<T> 返回结果, 如果不能够转换，将返回nullptr
     */
    template < typename T > std::shared_ptr< T > as() {
        if ( !is< T >() )
            return nullptr;

        return std::static_pointer_cast< T >( std::const_pointer_cast< Frame >( shared_from_this() ) );
    }

private:
    friend class Filter;
};

class OB_EXTENSION_API VideoFrame : public Frame {
public:
    VideoFrame( Frame& frame );
    virtual ~VideoFrame(){};

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
     * @brief 获取帧的元数据
     *
     * @return void* 返回帧的元数据
     */
    void* metadata();

    /**
     * @brief 获取帧的元数据大小
     *
     * @return uint32_t 返回帧的元数据大小
     */
    uint32_t metadataSize();
};

class OB_EXTENSION_API ColorFrame : public VideoFrame {
public:
    ColorFrame( Frame& frame );
    ~ColorFrame(){};
};

class OB_EXTENSION_API DepthFrame : public VideoFrame {
public:
    DepthFrame( Frame& frame );
    ~DepthFrame(){};

    /**
     * @brief 获取深度帧的值刻度，单位为 mm/step，
     *      如valueScale=0.1, 某坐标像素值为pixelValue=10000，
     *     则表示深度值value = pixelValue*valueScale = 10000*0.1=1000mm。
     *
     * @return float
     */
    float getValueScale();
};

class OB_EXTENSION_API IRFrame : public VideoFrame {
public:
    IRFrame( Frame& frame );
    virtual ~IRFrame(){};
};

class OB_EXTENSION_API FrameSet : public Frame {

public:
    // FrameSet();
    FrameSet( Frame& frame );
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
     * @return std::shared_ptr<DepthFrame> 返回深度帧
     */
    std::shared_ptr< DepthFrame > depthFrame();

    /**
     * @brief 获取彩色帧
     *
     * @return std::shared_ptr<ColorFrame> 返回彩色帧
     */
    std::shared_ptr< ColorFrame > colorFrame();

    /**
     * @brief 获取红外帧
     *
     * @return std::shared_ptr<IRFrame> 返回红外帧
     */
    std::shared_ptr< IRFrame > irFrame();

    /**
     * @brief 通过传感器类型获取帧
     *
     * @param frameType 传感器的类型
     * @return std::shared_ptr<Frame> 返回相应类型的帧
     */
    std::shared_ptr< Frame > getFrame( OBSensorType sensorType );

    friend class Pipeline;
    friend class Filter;
};

template < typename T > bool Frame::is() {
    switch ( this->type() ) {
    case OB_FRAME_IR:
        return ( typeid( T ) == typeid( IRFrame ) || typeid( T ) == typeid( VideoFrame ) );
    case OB_FRAME_DEPTH:
        return ( typeid( T ) == typeid( DepthFrame ) || typeid( T ) == typeid( VideoFrame ) );
    case OB_FRAME_COLOR:
        return ( typeid( T ) == typeid( ColorFrame ) || typeid( T ) == typeid( VideoFrame ) );
    case OB_FRAME_SET:
        return ( typeid( T ) == typeid( FrameSet ) );
    default:
        break;
    }
    return false;
}
}  // namespace ob

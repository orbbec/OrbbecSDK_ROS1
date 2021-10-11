/**
 * @file StreamProfile.hpp
 * @brief 流配置相关类型，用于获取流的宽、高、帧率及格式等信息
 *
 */
#pragma once

#include "Types.hpp"

#include <memory>

struct StreamProfileImpl;
struct StreamProfileListImpl;

namespace ob {

class OB_EXTENSION_API StreamProfile {
private:
    std::unique_ptr< StreamProfileImpl > impl_;

public:
    StreamProfile( std::unique_ptr< StreamProfileImpl > impl );
    ~StreamProfile();
    /**
     * @brief 获取流的帧率
     *
     * @return uint32_t 返回流的帧率
     */
    uint32_t fps();
    /**
     * @brief 获取流的宽
     *
     * @return uint32_t 返回流的宽
     */
    uint32_t width();
    /**
     * @brief 获取流的高
     *
     * @return uint32_t 返回流的高
     */
    uint32_t height();
    /**
     * @brief 获取流的格式
     *
     * @return OBFormat 返回流的格式
     */
    OBFormat format();
    /**
     * @brief 获取流的类型
     *
     * @return OBStreamType 返回流的类型
     */
    OBStreamType type();

    friend class Sensor;
};

class OB_EXTENSION_API StreamProfileList {
protected:
    std::unique_ptr< StreamProfileListImpl > impl_;

public:
    StreamProfileList( std::unique_ptr< StreamProfileListImpl > impl );
    ~StreamProfileList();

    /**
     * @brief 获取StreamProfile数量
     *
     * @return uint32_t 返回StreamProfile的数量
     */
    uint32_t count();

    /**
     * @brief 通过索引号获取StreamProfile
     *
     * @param index 要创建设备的索，范围 [0, count-1]，如果index超出范围将抛异常
     * @return std::shared_ptr<StreamProfile> 返回StreamProfile对象
     */
    const std::shared_ptr< StreamProfile > getProfile( uint32_t index );
};
}  // namespace ob

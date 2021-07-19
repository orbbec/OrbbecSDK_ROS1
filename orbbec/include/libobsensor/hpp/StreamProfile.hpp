/**
 * @file StreamProfile.hpp
 * @brief 流配置相关类型，用于获取流的宽、高、帧率及格式等信息
 * 
 */
#pragma once

#include "libobsensor/hpp/Types.hpp"
#include <memory>

struct StreamProfileImpl;

namespace ob
{
    class OB_EXTENSION_API StreamProfile
    {
    protected:
        std::unique_ptr<StreamProfileImpl> impl_;
    public:
   
        StreamProfile(std::unique_ptr<StreamProfileImpl> impl);
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
         * @return ob_format 返回流的格式
         */
        ob_format format();
        /**
         * @brief 获取流的类型
         * 
         * @return ob_stream_type 返回流的类型
         */
        ob_stream_type type();

        //template<class T>
        //bool is() const
        //{
        //    T extension(*this);
        //    return extension;
        //}

        //template<class T>
        //T as() const
        //{
        //    T extension(*this);
        //    return extension;
        //}

        friend class Sensor;
    };

    class OB_EXTENSION_API ImuStreamProfile :  public StreamProfile {
        
        public:
      
        /**
         * @brief 获取加速度的频率
         *
         * @return OB_ACCEL_ODR_EM 返回加速度的频率
         */
        OB_ACCEL_ODR_EM accelFps();
        /**
         * @brief 获取加速度量程
         *
         * @return OB_ACCEL_FS_EM 返回加速度量程
         */
        OB_ACCEL_FS_EM accelRange();
        /**
         * @brief 获取陀螺仪频率
         *
         * @return OB_GYRO_ODR_EM 返回陀螺仪频率
         */
        OB_GYRO_ODR_EM gyroFps();
        /**
         * @brief 获取陀螺仪量程
         *
         * @return OB_GYRO_FS_EM 返回陀螺仪量程
         */
        OB_GYRO_FS_EM gyroRange();
        /**
         * @brief 获取流的格式
         *
         * @return ob_format 返回流的格式
         */
        ob_format format();
        /**
         * @brief 获取流的类型
         *
         * @return ob_stream_type 返回流的类型
         */
        ob_stream_type type();
    };
 } // namespace ob

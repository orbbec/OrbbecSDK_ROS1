/**
 * @file Version.hpp
 * @brief 获取SDK版本号相关信息的类
 * 
 */
#include "libobsensor/hpp/Types.hpp"
#include "libobsensor/ObSensor.h"

namespace ob
{
    class Version
    {
    public:
        /**
         * @brief 获取SDK主版本号
         * 
         * @return int 返回SDK主版本号
         */
        int major()
        {
            return OB_API_VERSION / 10000;
        }
        /**
         * @brief 获取SDK副版本号
         * 
         * @return int 返回SDK副版本号
         */
        int minor()
        {
            return ( OB_API_VERSION % 10000 ) / 100;
        }
        /**
         * @brief 获取SDK修订版本号
         * 
         * @return int 返回SDK修订版本号
         */
        int patch()
        {
            return OB_API_VERSION % 100;
        }
        /**
         * @brief 获取SDK版本号
         * 
         * @return int 返回SDK版本号
         */
        int version()
        {
            return OB_API_VERSION;
        }
    };
}
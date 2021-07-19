/**
 * @file Version.h
 * @brief 获取SDK版本号相关信息函数
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"
#include "../ObSensor.h"

/**
 * @brief 获取SDK版本号
 * 
 * @return int 返回SDK版本号
 */
int ob_get_version()
{
    return OB_API_VERSION;
}

/**
 * @brief 获取SDK主版本号
 * 
 * @return int 返回SDK主版本号
 */
int ob_get_major_version()
{
    return OB_API_VERSION / 10000;
}

/**
 * @brief 获取SDK副版本号
 * 
 * @return int 返回SDK副版本号
 */
int ob_get_minor_version()
{
    return ( OB_API_VERSION % 10000 ) / 100;
}

/**
 * @brief 获取SDK修订版本号
 * 
 * @return int 返回SDK修订版本号
 */
int ob_get_patch_version()
{
    return OB_API_VERSION % 100;
}

#ifdef __cplusplus
}
#endif
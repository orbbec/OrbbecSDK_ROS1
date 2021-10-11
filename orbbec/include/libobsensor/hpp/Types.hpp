#pragma once
/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2020  Orbbec Corporation. All Rights Reserved. */

/**   \file Types.hpp
 *    \brief  提供SDK的结构体、枚举常量定义（依赖libobsensor/h/ObTypes.h）
 *
 *
 */
#pragma once

#include "libobsensor/h/ObTypes.h"

#include <functional>

#ifdef __cplusplus
extern "C" {
#endif

using SendFileCallback           = std::function< void( OBFileTranState state, const char* message, uint8_t percent ) >;
using DeviceUpgradeCallback      = std::function< void( OBUpgradeState state, const char* message, uint8_t percent ) >;
using DeviceStateChangedCallback = std::function< void( OBDeviceState state ) >;

/**
 * @brief 获取raw data属性数据时数据及进度回调
 *
 * @param dataChunk 数据块
 * @param state 获取数据状态
 */
using GetDataCallback = std::function< void( OBDataTranState state, OBDataChunk* dataChunk ) >;

/**
 * @brief 设置raw data属性数据时进度回调
 *
 * @param percent  进度百分比
 * @param state  设置数据状态
 */
using SetDataCallback = std::function< void( OBDataTranState state, uint8_t percent ) >;

#ifdef __cplusplus
}
#endif

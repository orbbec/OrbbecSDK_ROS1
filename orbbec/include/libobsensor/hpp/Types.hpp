#pragma once
/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2020  Orbbec Corporation. All Rights Reserved. */

/**   \file Types.hpp
 *    \brief  提供SDK的结构体、枚举常量定义
 *   
 *
 */
#pragma once

#include "libobsensor/h/ObTypes.h"
#include "libobsensor/h/ObPropertyTypes.h"

#include <functional>
#include <memory>

#ifdef __cplusplus
extern "C" {
#endif

typedef ob_version OBVersion;
typedef ob_status OBStatus;
typedef ob_log_severity OBLogServerity;
typedef ob_log_out_type OBLogOutType;
typedef ob_exception_type OBExceptionType;
//typedef ob_global_unified_property OBGlobalUnifiedProperty;
//typedef ob_sensor_property OBSensorProperty;
typedef ob_sensor_type OBSensorType;
//typedef ob_property_range OBPropertyRange;
typedef ob_stream_type OBStreamType;
typedef ob_frame_type OBFrameType;
typedef ob_format OBFormat;
typedef ob_orientation_type OBOrientationType;

typedef DEVICE_STATE OBDeviceState;

using SendFileCallback = std::function<void(FileTranState state, const char* message, uint8_t percent)>;
using DeviceUpgradeCallback = std::function<void(UpgradeState state, const char* message, uint8_t percent)>;
using DeviceStateChangedCallback = std::function<void(OBDeviceState state)>;

/**
 * @brief 获取raw data属性数据时数据及进度回调
 * 
 * @param dataChunk 数据块
 * @param state 获取数据状态
 */
using GetDataCallback = std::function<void(DataTranState state, DataChunk *dataChunk)>;
    
 /**
  * @brief 设置raw data属性数据时进度回调
  *
  * @param percent  进度百分比
  * @param state  设置数据状态
  */
using SetDataCallback = std::function< void(DataTranState state,  uint8_t percent) >;


#ifdef __cplusplus
}
#endif

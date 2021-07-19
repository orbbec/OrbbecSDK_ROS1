// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

/**
 * @file ObPropertyTypes.h
 * @brief 提供SDK的结构体、枚举常量定义
 */


#pragma once

#if (defined(WIN32) || defined(_WIN32) || defined(WINCE))
#  ifdef OB_EXPORTS
#    define OB_EXTENSION_API __declspec(dllexport)
#  else
#    define OB_EXTENSION_API __declspec(dllimport)
#  endif
#else
#  define OB_EXTENSION_API
#endif

#ifdef __cplusplus
    extern "C" {
#endif
#include<stdint.h>

#pragma pack(1)  // struct 1-byte align

#define OB_DEVICE_PROPERTY_BEGIN 0
#define OB_DEVICE_PROPERTY_END 999
#define OB_DATA_TYPE_BEGIN 1000
#define OB_DATA_TYPE_END 1999
#define OB_SENSOR_PROPERTY_BEGIN 2000
#define OB_SENSOR_PROPERTY_END 2999
#define OB_SDK_PROPERTY_BEGIN 3000
#define OB_SDK_PROPERTY_END 3499
#define OB_SDK_DATA_TYPE_BEGIN 3500
#define OB_SDK_DATA_TYPE_END 3999
#define OB_RAW_DATA_BEGIN 4000
#define OB_RAW_DATA_END 4999
#define OB_DEVICE_DEBUG_PROPERTY_BEGIN 5000
#define OB_DEVICE_DEBUG_PROPERTY_END 5499
#define OB_DEVICE_DEBUG_DATA_TYPE_BEGIN 5500
#define OB_DEVICE_DEBUG_DATA_TYPE_END 5999

/**
 * @brief 描述设备的属性的枚举值
 * 
 */
typedef enum {
    // 0~999为设备端int, bool及float类型控制命令
    // Device:: get/setxxxProperty、 getxxxPropertyRange (xxx表示数据类型)
    OB_DEVICE_PROPERTY_FLASH_WRITE_PROTECTION_BOOL                          = 0,  // FLASH写保护开关；0，关闭；1，开启
    OB_DEVICE_PROPERTY_TEC_ENABLE_BOOL                                      = 1,  // TEC开关
    OB_DEVICE_PROPERTY_LDP_BOOL                                             = 2,  // LDP开关
    OB_DEVICE_PROPERTY_EMITTER_BOOL                                         = 3,  // 激光开光
    OB_DEVICE_PROPERTY_LASER_PULSE_WIDTH_INT                                = 4,  // 读写激光Time值（脉宽）,
    OB_DEVICE_PROPERTY_LASER_CURRENT_FLOAT                                  = 5,  // 激光电流
    OB_DEVICE_PROPERTY_FLOOD_BOOL                                           = 6,  // 泛光灯开关
    OB_DEVICE_PROPERTY_FLOOD_LEVEL_INT                                      = 7,  // 泛光灯等级
    OB_DEVICE_PROPERTY_TEMPERATURE_COMPENSATION_ENABLE_BOOL                 = 8,  // 温度补偿开关
    OB_DEVICE_PROPERTY_TEMPERATURE_CALIBRATED_IR_FLOAT                      = 9,  // IR标定温度
    OB_DEVICE_PROPERTY_TEMPERATURE_CALIBRATED_LDMP_FLOAT                    = 10,  // 激光标定温度
    OB_DEVICE_PROPERTY_TEMPERATURE_COMPENSATION_COEFFICIENT_IR_FLOAT        = 11,  // IR温补系数
    OB_DEVICE_PROPERTY_TEMPERATURE_COMPENSATION_COEFFICIENT_LDMP_FLOAT      = 12,  // 激光温补系数
    OB_DEVICE_PROPERTY_DEPTH_ALIGN_SOFTWARE_BOOL                            = 13,  // 软件D2C
    OB_DEVICE_PROPERTY_DEPTH_MIRROR_BOOL                                    = 14,  // 深度镜像
    OB_DEVICE_PROPERTY_DEPTH_FLIP_BOOL                                      = 15,  // 深度翻转
    OB_DEVICE_PROPERTY_DEPTH_POSTFILTER_BOOL                                = 16,  // 深度Postfilter
    OB_DEVICE_PROPERTY_DEPTH_HOLEFILTER_BOOL                                = 17,  // 深度Holefilter
    OB_DEVICE_PROPERTY_IR_MIRROR_BOOL                                       = 18,  // IR 镜像
    OB_DEVICE_PROPERTY_IR_FLIP_BOOL                                         = 19,  // IR 翻转
    OB_DEVICE_PROPERTY_IR_SWITCH_BOOL                                       = 20,  // IR 左右切换
    OB_DEVICE_PROPERTY_HARDWARE_SYNC_BOOL                                   = 21,  // 开启硬件同步
    OB_DEVICE_PROPERTY_MIN_DEPTH_INT                                        = 22,  // 最小深度阈值
    OB_DEVICE_PROPERTY_MAX_DEPTH_INT                                        = 23,  // 最大深度阈值
    OB_DEVICE_PROPERTY_DEPTH_SOFT_FILTER_BOOL                               = 24,  // 软件滤波开关
    OB_DEVICE_PROPERTY_ZERO_PLANE_DISTANCE_FLOAT                            = 25,  // ZPD
    OB_DEVICE_PROPERTY_ZERO_PLANE_PIXEL_SIZE_FLOAT                          = 26,  // ZPPS
    OB_DEVICE_PROPERTY_CHIP_TYPE_INT                                        = 27,  // 芯片类型
    OB_DEVICE_PROPERTY_USB_SPEED_INT                                        = 28,  // usb speed
    OB_DEVICE_PROPERTY_SOFT_RESET_BOOL                                      = 29,  // 软件复位
    OB_DEVICE_PROPERTY_LDP_THRES_UP_INT                                     = 30,  // LDP阈值的上限
    OB_DEVICE_PROPERTY_LDP_THRES_LOW_INT                                    = 31,  // LDP阈值的下限
    OB_DEVICE_PROPERTY_LDP_STATUS_BOOL                                      = 32,  // LDP状态
    OB_DEVICE_PROPERTY_LASER_TEMPERATURE_FLOAT                              = 33,  // 获取激光温度
    OB_DEVICE_PROPERTY_BOOTLOADER_WRITE_PROTECTION_STATUS_BOOL              = 34,  // Bootloader区flash写保护状态
    OB_DEVICE_PROPERTY_RT_IR_TEMP_FLOAT                                     = 35,  // 获取实时IR温度
    OB_DEVICE_PROPERTY_RT_LDMP_TEMP_FLOAT                                   = 36,  // 获取实时LDMP温度
    OB_DEVICE_PROPERTY_RT_RGB_TEMP_FLOAT                                    = 37,  // 获取实时RGB温度
    OB_DEVICE_PROPERTY_STOP_DEPTH_STREAM_BOOL                               = 38,  // 关闭深度流，用于无法通过标准UVC协议关流的设备
    OB_DEVICE_PROPERTY_STOP_IR_STREAM_BOOL                                  = 39,  // 关闭IR流用于无法通过标准UVC协议关流的设备
    OB_DEVICE_PROPERTY_DEPTH_MAX_DIFF_INT                                   = 40,  // soft filter maxdiff param
    OB_DEVICE_PROPERTY_DEPTH_MAX_SPECKLE_SIZE_INT                           = 41,  // soft filter maxSpeckleSize
    OB_DEVICE_PROPERTY_DEPTH_ALIGN_HARDWARE_BOOL                            = 42,  // 硬件d2c开
    OB_DEVICE_PROPERTY_TIMESTAMP_OFFSET_INT                                 = 43,  // 时间戳调校
    OB_DEVICE_PROPERTY_ORIENTATION_SWITCH_BOOL                              = 44,  // 横竖屏切换, 0:竖屏，1横屏
    OB_DEVICE_PROPERTY_ENABLE_CALIBRATION_BOOL                              = 45,  // 是否需要裁剪
    // OB_DEVICE_PROPERTY_TOF_FPS_INT                                          = 46,  // TOF帧率
    OB_DEVICE_PROPERTY_TOF_EXPOSURE_TIME_INT                                = 47,  // TOF曝光时间
    OB_DEVICE_PROPERTY_TOF_GAIN_INT                                         = 48,  // TOF增益
    OB_DEVICE_PROPERTY_TOF_MIRROR_INT                                       = 49,  // TOF镜像开光, 0: close 1:H-Mirror  2: V-Mirror 3:H-V-Mirror
    OB_DEVICE_PROPERTY_TOF_GAUSSIAN_FILTER_BOOL                             = 50,  // 噪声滤波开关
    OB_DEVICE_PROPERTY_TOF_SCATTER_FILTER_BOOL                              = 51,  // 散射滤波开关
    OB_DEVICE_PROPERTY_TOF_BILATERAL_FILTER_BOOL                            = 52,  // 双边滤波开关
    OB_DEVICE_PROPERTY_TOF_FLY_POINT_FILTER_BOOL                            = 53,  // 点云滤波开关
    OB_DEVICE_PROPERTY_TOF_MEDIAN_FILTER_BOOL                               = 54,  // 中值滤波开关
    OB_DEVICE_PROPERTY_TOF_CONFIDENCE_FILTER_BOOL                           = 55,  // 置信滤波开关
    OB_DEVICE_PROPERTY_TOF_SHUFFLE_MODE_BOOL                                = 56,  // TOF Phase Shuffle模式
    OB_DEVICE_PROPERTY_REBOOT_DEVICE_BOOL                                   = 57,  // 设备重启
    OB_DEVICE_PROPERTY_FACTORY_RESET_BOOL                                   = 58,  // 恢复出厂设置  
    OB_DEVICE_PROPERTY_IR_MODE_SWITCH_BOOL                                  = 59,  // IR模式 散斑(false)/纯净图(true) 切换
    OB_DEVICE_PROPERTY_FRAME_RATE_MODE_SWITCH_BOOL                          = 60,  // 帧率 固定模式(false)/动态调整模式(true) 切换
    OB_DEVICE_PROPERTY_HARDWARE_DISTORTION_SWITCH_BOOL                      = 61,  // 硬件去畸变开关	
	OB_DEVICE_PROPERTY_FAN_WORK_MODE_INT                                    = 62,  // 风扇开关模式                    
    OB_DEVICE_PROPERTY_DEPTH_ALIGN_HARDWARE_MODE_INT                        = 63,  // 多分辨率D2C模式 
    OB_DEVICE_PROPERTY_ANTI_COLLUSION_ACTIVATION_STATUS_BOOL                = 64,  // 防串货状态
    OB_DEVICE_PROPERTY_SOFTWARE_DISTORTION_SWITCH_BOOL                      = 65,  // 软件去畸变开关 OBBoolPropertyRangeTran
    OB_DEVICE_PROPERTY_SYNC_MODE_INT                                        = 66,  // (倚天剑已弃用)同步模式 OBIntPropertyRangeTran  
    OB_DEVICE_PROPERTY_SYNC_EXT_OUT_DELAY_TIME_INT                          = 67,  // (倚天剑已弃用)配置同步信号延时us数后再triggler out输出 OBIntPropertyRangeTran
    OB_DEVICE_PROPERTY_IR_FRAME_RATE_INT                                    = 68,  // IR帧率设置，用于无法通过标准UVC协议设置帧率的设备
    OB_DEVICE_PROPERTY_DEPTH_FRAME_RATE_INT                                 = 69,  // Depth帧率设置，用于无法通过标准UVC协议设置帧率的设备
    OB_DEVICE_PROPERTY_TEC_MAX_CURRENT_INT                                  = 70,  // TEC 最大电流 -> 百分比 0~100%
    OB_DEVICE_PROPERTY_TEC_MAX_CURRENT_CONFIG_INT                           = 71,  // TEC最大电流配置（掉电保存）
    OB_DEVICE_PROPERTY_FAN_WORK_MODE_CONFIG_INT                             = 72,  // 风扇开关模式配置（掉电保存）
    OB_DEVICE_PROPERTY_DEPTH_BIT_PER_PIXEL_INT                              = 73,  // Depth像素位数配置， 8/12/14/16 等，单位为bpp。实际图像帧中每像素占用的位数可以大于实际像素位数（如16bit深度图，有效像素位数为10bit）
    OB_DEVICE_PROPERTY_IR_BIT_PER_PIXEL_INT                                 = 74,  // IR像素位数配置， 8/12/14/16 等，单位为bpp
    OB_DEVICE_PROPERTY_DEPTH_UNIT_INT                                       = 75,  // 深度单位，mx6600: 0: 0.1mm, 1: 0.2mm, 2: 0.4mm, 3: 0.8mm, 4: 1.6mm -> 2^n/10mm
    OB_DEVICE_PROPERTY_TOF_FILTER_RANGE_INT                                 = 76,  // tof滤波场景范围配置 TOF_FILTER_RANGE
    
    // 1000~1999为设备端结构体控制命令
    // Device:: get/setStructedData
    OB_DATA_TYPE_VERSIONS                                                   = 1000,  // 版本信息
    OB_DATA_TYPE_CAMERA_PARA                                                = 1001,  // 相机内外参数
    OB_DATA_TYPE_BASELINE_CALIBRATION_PARA                                  = 1002,  // 基线标定参数
    OB_DATA_TYPE_DEVICE_TEMPERATURE                                         = 1003,  // 设备温度信息
    OB_DATA_TYPE_DEVICE_AE_PARAMS                                           = 1004,  // AE调试参数
    OB_DATA_TYPE_EXTENSION_PARAMS                                           = 1005,  // 扩展参数
    OB_DATA_TYPE_DEVICE_UPGRADE_STATUS                                      = 1006,  // 固件升级状态 read only
    OB_DATA_TYPE_DEVICE_CALIBRATION_UPGRADE_STATUS                          = 1007,  // 标定文件升级状态 read only
    OB_DATA_TYPE_DEVICE_FILE_TRAN_STATUS                                    = 1008,  // 文件传输状态 read only
    OB_DATA_TYPE_PTZ_CONTROL                                                = 1009,  // 云台控制
    OB_DATA_TYPE_DIGITAL_ZOOM                                               = 1010,  // 数字变焦
    OB_DATA_TYPE_TOF_TX_RX_TEMP                                             = 1011,  // TOF tx rx温度
    OB_DATA_TYPE_TOF_MODULATION_FREQ                                        = 1012,  // TOF调制频率信息
    OB_DATA_TYPE_TOF_DUTY_CYCLE                                             = 1013,  // TOF调制信号占空比信息
    OB_DATA_TYPE_TOF_CALIBRATION_PARA                                       = 1014,  // TOF标定参数
    OB_DATA_TYPE_TOF_DEPTH_COEF_PARA                                        = 1015,  // TOF距离转深度系数
    OB_DATA_TYPE_TOF_VCSEL_TEMP_COMPENSATION                                = 1016,  // TOF温补系数
    OB_DATA_TYPE_TOF_GAUSSIAN_FILTER_PARA                                   = 1017,  // TOF高斯噪声滤波参数
    OB_DATA_TYPE_TOF_SCATTER_FILTER_PARA                                    = 1018,  // TOF散射滤波参数
    OB_DATA_TYPE_TOF_BILATERAL_FILTER_PARA                                  = 1019,  // TOF双边滤波参数
    OB_DATA_TYPE_TOF_FLY_POINT_FILTER_PARA                                  = 1020,  // TOF点云滤波参数
    OB_DATA_TYPE_TOF_MEDIAN_FILTER_PARA                                     = 1021,  // TOF中值滤波参数
    OB_DATA_TYPE_TOF_CONFIDENCE_FILTER_PARA                                 = 1022,  // TOF置信滤波参数
    OB_DATA_TYPE_TOF_NEAREST_FARTHEST_LENGTH                                = 1023,  // TOF最近与最远距离
    OB_DATA_TYPE_TOF_EXPOSURE_THRESHOLD_CONTROL                             = 1024,  // TOF曝光阈值范围
    OB_DATA_TYPE_DEVICE_STATE                                               = 1025,  // 获取当前设备状态
    OB_DATA_TYPE_TEC_DATA                                                   = 1026,  // 获取TEC数据
    OB_DATA_TYPE_GPM_CONFIG_DATA                                            = 1027,  // GPM配置数据，包括16个坐标和相关的阈值
    OB_DATA_TYPE_GPM_STATUS_DATA                                            = 1028,  // 获取GPM状态数据,包括16个点机及其统计信息
    OB_DATA_TYPE_ANTI_COLLUSION_ACTIVATION_CONTENT                          = 1029,  // 防串货激活码读写
    OB_DATA_TYPE_ANTI_COLLUSION_ACTIVATION_VERIFY                           = 1030,  // 防串货激活码验证
    OB_DATA_TYPE_GET_GYRO_PRESETS_ODR_LIST                                  = 1031,  // 获取陀螺仪支持的采样率列表
    OB_DATA_TYPE_GET_ACCEL_PRESETS_ODR_LIST                                 = 1032,  // 获取加速度计支持的采样率列表
    OB_DATA_TYPE_GET_GYRO_PRESETS_FULL_SCALE_LIST                           = 1033,  // 获取陀螺仪支持的量程列表
    OB_DATA_TYPE_GET_ACCEL_PRESETS_FULL_SCALE_LIST                          = 1034,  // 获取加速度计支持的量程列表
    OB_DATA_TYPE_DEVICE_SERIAL_NUMBER                                       = 1035,  // get/set序列号
    OB_DATA_TYPE_DEVICE_PRODUCT_NUMBER                                      = 1036,  // get/set PN
    OB_DATA_TYPE_DEVICE_TIME                                                = 1037,  // get/set device time
    OB_DATA_TYPE_TOF_MULTI_DEVICE_SYNC_CONFIG                               = 1038,  // 多设备同步模式和参数配置
    OB_DATA_TYPE_TEMP_COMPENSATE_PARA                                       = 1039,  // get/set 温度补偿系数

    // 2000~2999为Sensor控制命令
    // Device:: get/setxxxProperty、 getxxxPropertyRange (xxx表示数据类型)
    OB_SENSOR_PROPERTY_ENABLE_AUTO_EXPOSURE_BOOL                            = 2000,  // 自动曝光
    OB_SENSOR_PROPERTY_EXPOSURE_INT                                         = 2001,  // 曝光调节
    OB_SENSOR_PROPERTY_GAIN_INT                                             = 2002,  // 增益调节
    OB_SENSOR_PROPERTY_ENABLE_AUTO_WHITE_BALANCE_BOOL                       = 2003,  // 自动白平衡
    OB_SENSOR_PROPERTY_WHITE_BALANCE_INT                                    = 2004,  // 白平衡调节
    OB_SENSOR_PROPERTY_BRIGHTNESS_INT                                       = 2005,  // 亮度调节
    OB_SENSOR_PROPERTY_SHARPNESS_INT                                        = 2006,  // 锐度调节
    // 用白平衡调节代替色温调节
    // OB_SENSOR_PROPERTY_COLOR_TEMPERATURE_INT                                = 2007,  // 色温调节
    OB_SENSOR_PROPERTY_SATURATION_INT                                       = 2008,  // 饱和度调节
    OB_SENSOR_PROPERTY_CONTRAST_INT                                         = 2009,  // 对比度调节
    OB_SENSOR_PROPERTY_GAMMA_INT                                            = 2010,  // 伽马值调节
    OB_SENSOR_PROPERTY_ROLL_INT                                             = 2011,  // 图像旋转
    OB_SENSOR_PROPERTY_AUTO_EXPOSURE_PRIORITY_INT                           = 2012,  // 自动曝光优先
    OB_SENSOR_PROPERTY_BACKLIGHT_COMPENSATION_INT                           = 2013,  // 亮度补偿
    OB_SENSOR_PROPERTY_HUE_INT                                              = 2014,  // 彩色色调
    OB_SENSOR_PROPERTY_POWER_LINE_FREQUENCY_INT                             = 2015,  // 电力线路频率
    // MIRROR 和 FLIP 通过私有协议控制命令实现
    // OB_SENSOR_PROPERTY_MIRROR_BOOL                                          = 2016,  // 图像镜像
    // OB_SENSOR_PROPERTY_FLIP_BOOL                                            = 2017,  // 图像翻转
    // 以上Sensor控制命令会被转换成标准UVC控制协议命令
    // OB_SENSOR_PROPERTY_ID_INT                                               = 2018,  // SensorID
    OB_SENSOR_PROPERTY_GYRO_SWITCH_BOOL                                     = 2019,  // 陀螺仪开关
    OB_SENSOR_PROPERTY_ACCEL_SWITCH_BOOL                                    = 2020,  // 加速度计开关
    OB_SENSOR_PROPERTY_GYRO_ODR_INT                                         = 2021,  // get/set当前陀螺仪的采样率
    OB_SENSOR_PROPERTY_ACCEL_ODR_INT                                        = 2022,  // get/set当前加速度计的采样率
    OB_SENSOR_PROPERTY_GYRO_FULL_SCALE_INT                                  = 2023,  // get/set当前陀螺仪的量程
    OB_SENSOR_PROPERTY_ACCEL_FULL_SCALE_INT                                 = 2024,  // get/set当前加速度计的量程

    // 3000~3499为SDK int, bool及float类型控制命令
    // Device:: get/setxxxProperty、 getxxxPropertyRange (xxx表示数据类型)
    OB_SDK_PROPERTY_DEPTH_SOFT_FILTER_BOOL                                  = 3000,  // 软件滤波开关
    OB_SDK_PROPERTY_DEPTH_MAX_DIFF_INT                                      = 3001,  // soft filter maxdiff param
    OB_SDK_PROPERTY_DEPTH_MAX_SPECKLE_SIZE_INT                              = 3002,  // soft filter maxSpeckleSize
    OB_SDK_PROPERTY_SOFT_FILTER_TYPE_BOOL                                   = 3003,  // soft filter filterType ,0-disparity 1-depth
    OB_SDK_PROPERTY_DISPARITY_TO_DEPTH_BOOL                                 = 3004,  // convert disparity to depth
    OB_SDK_PROPERTY_IR_MIRROR_BOOL                                          = 3005,  // SDK ir mirror

    // 3500~3999为SDK结构体控制命令
    OB_SDK_DATA_DEPTH_VALUE_LIMIT_RANGE                                     = 3500,  // DEPTH VALUE LIMIT(深度最大值、最小值, unit: mm) 

    //4000~4999为RawData控制命令
    // Device:: get/setRawData
    OB_RAW_DATA_MULTIPLE_DISTANCE_CALIBRATION_PARA                          = 4000,  // 多距离标定参数
    OB_RAW_DATA_REFERENCE_IMAGE                                             = 4001,  // 参考图
    OB_RAW_DATA_CAMERA_CFG_PARAMETER_960_1280                               = 4002,  // 对应960_1280分辨率的相机配置参数 write only
    OB_RAW_DATA_CAMERA_CFG_PARAMETER_720_1280                               = 4003,  // 对应720_1280分辨率的相机配置参数  write only
    OB_RAW_DATA_CAMERA_CFG_PARAMETER_1280_720                               = 4004,  // 对应720_1280分辨率的相机配置参数  write only
    OB_RAW_DATA_HARDWARE_ALIGN_PARA                                         = 4005,  // 硬件对齐参数
    OB_RAW_DATA_SOFTWARE_ALIGN_PARA                                         = 4006,  // 软件对齐参数
    OB_RAW_DATA_HARDWARE_DISTORTION_PARA                                    = 4007,  // 硬件去畸变参数
    OB_RAW_DATA_DEPTH_CONFIG_PARA                                           = 4008,  // Config区
    OB_RAW_DATA_HARDWARE_ALIGN_PARA_0                                       = 4009,  // 硬件对齐参数0, 适用与多分辨率对齐场景，由设备决定HARDWARE_ALIGN_PARA_xxx 与分辨率的映射关系
    OB_RAW_DATA_HARDWARE_ALIGN_PARA_1                                       = 4010,  // 硬件对齐参数1
    OB_RAW_DATA_HARDWARE_ALIGN_PARA_2                                       = 4011,  // 硬件对齐参数2  
    OB_RAW_DATA_HARDWARE_ALIGN_PARA_3                                       = 4012,  // 硬件对齐参数3
    OB_RAW_DATA_HARDWARE_ALIGN_PARA_4                                       = 4013,  // 硬件对齐参数4  
    OB_RAW_DATA_HARDWARE_ALIGN_PARA_5                                       = 4014,  // 硬件对齐参数5
    OB_RAW_DATA_TEMP_COMPENSATE_PARA                                        = 4015,  // 温补参数    
    OB_RAW_DATA_SOFTWARE_ALIGN_PARA_0                                       = 4016,  // 软件对齐参数0, 适用与多分辨率对齐场景，由设备决定HARDWARE_ALIGN_PARA_xxx 与分辨率的映射关系
    OB_RAW_DATA_SOFTWARE_ALIGN_PARA_1                                       = 4017,  // 软件对齐参数1
    OB_RAW_DATA_SOFTWARE_ALIGN_PARA_2                                       = 4018,  // 软件对齐参数2  
    OB_RAW_DATA_SOFTWARE_ALIGN_PARA_3                                       = 4019,  // 软件对齐参数3
    OB_RAW_DATA_SOFTWARE_ALIGN_PARA_4                                       = 4020,  // 软件对齐参数4  
    OB_RAW_DATA_SOFTWARE_ALIGN_PARA_5                                       = 4021,  // 软件对齐参数5

    //5000~5499为调试用int, bool及float类型控制命令
    // Device:: get/setxxxProperty、 getxxxPropertyRange (xxx表示数据类型)
    OB_DEVICE_DEBUG_ADB_FUNCTION_CONTROL_BOOL                               = 5000,  // ADB调试功能开关
    OB_DEVICE_DEBUG_SET_FORCE_UPGRADE_BOOL                                  = 5001,  // 强制升级
    OB_DEVICE_DEBUG_MX6300_START_TIME_INT                                   = 5002,  // 获取MX6300固件启动时间

    // 5500~5999为调试用结构体控制命令
    // Device:: get/setStructedData
    OB_DEVICE_DEBUG_RECORD_RGB_DATA                                     = 5500,  // 设备端 RGB 传图控制（调试功能）
    OB_DEVICE_DEBUG_RECORD_PHASE_DATA                                   = 5501,  // 设备端 raw Phase 传图控制（调试功能）
    OB_DEVICE_DEBUG_RECORD_IR_DATA                                      = 5502,  // 设备端 IR 传图控制（调试功能）
    OB_DEVICE_DEBUG_RECORD_DEPTH_DATA                                   = 5503,  // 设备端 depth 传图控制（调试功能）

} OBGlobalUnifiedProperty, ob_global_unified_property;


/*
* 以下三个OBxxxxPropertyRangeTran 结构体，用于SDK与设备固件property数据传输
* 其成员变量数据类型的长度（4byte）遵循Property命令协议
*/

// 整形范围数据传输结构体
typedef struct 
{
    int32_t cur;
    int32_t max;
    int32_t min;
    int32_t step;
    int32_t def;
} OBIntPropertyRangeTran, ob_int_property_range_tran;

//获取浮点型数据传输结构体
typedef struct 
{
    float cur;
    float max;
    float min;
    float step;
    float def;
} OBFloatPropertyRangeTran, ob_float_property_range_tran;

// bool型范围数据传输结构体 (根据SDK与设备porperty类型数据通信协议，
// 必须保证结构体内每个字段都为4字节, 所以这里将各字段类型定义为uint32_t,
// uint32_t与bool类型转换遵循C++/C 语言标准规则)
typedef struct 
{
    uint32_t cur;
    uint32_t max;
    uint32_t min;
    uint32_t step;
    uint32_t def;
} OBBoolPropertyRangeTran, ob_bool_property_range_tran;


typedef struct 
{
    char firmwareVersion[16];      // 如：1.2.18
    char hardwareVersion[16];      // 如：1.0.18
    char sdkVersion[16];           // 支持最低sdk的版本号，sdk版本号：2.3.2（主.次.修订)         
    char depthChip[16];            // mx6000, mx6600                    
    char systemChip[16];           // ar9201
    char serialNumber[16];         // 序列号  
    int32_t  deviceType;           // 单目，双目，tof。枚举值
    char  deviceName[16];          // 设备名称 astra+
    char subSystemVersion[16];     // 如：倚天剑的MCU固件版本 1.0.23
    char resever[32];              // 保留
} VERSIONS;

typedef struct 
{
    float d_intr_p[4];   // 深度相机内参：[fx,fy,cx,cy]
    float c_intr_p[4];   // 彩色相机内参：[fx,fy,cx,cy]
    float d2c_r[9];      // 深度相机到彩色相机的旋转矩阵 [r00,r01,r02;r10,r11,r12;r20,r21,r22]
    float d2c_t[3];      // 深度相机到彩色相机的平移矩阵 [t1,t2,t3]
    float d_k[5];        // 深度相机畸变参数 [k1,k2,p1,p2,k3]  // 注意k3的位置，这个是算法定的，有些代码k3排在k2后边
    float c_k[5];        // 彩色相机畸变参数 [k1,k2,p1,p2,k3]
    uint32_t c_img_size[2]; // 彩色标定分辨率 [color_width, color_height]
    uint32_t d_img_size[2]; // 深度标定分辨率 [ir_width, ir_height]
} CAMERA_PARA;

typedef struct 
{
    float fBaseline;
    float fZ0;
} BASELINE_CALIBRATION_PARA;

typedef struct 
{
    float cpuTemp;
    float irTemp;
    float ldmTemp;
    float mainBoardTemp;
    float tecTemp;
    float imuTemp;
} DEVICE_TEMPERATURE;

typedef struct  {
    uint32_t min;
    uint32_t max;
} RANGE, DEPHT_VALUE_LIMIT_RANGE;

typedef struct  {
    RANGE    expTime;
    RANGE    AGain;
    RANGE    laserCurrent;
    uint32_t targetBrightness;
    uint32_t targetThd;
    uint32_t centerWeight;
    uint32_t skipFrame;
    uint32_t smoothSteps;
    uint32_t delay_ms;
    uint32_t meterMethod;
    uint8_t  expTimeAdj;
    uint8_t  AGainAdj;
    uint8_t  laserCurrentAdj;
    uint8_t  reserve;
} DEVICE_AE_PARAMS;

//typedef struct  {
//
//} EXTENSION_PARAMS;

typedef struct  {
    int16_t    state;
    int16_t    percentage;
    uint8_t  message[256];
} DEVICE_UPGRADE_STATUS;

typedef struct  {
    int32_t enable;
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
    float speed;
} PTZ_CONTROL;

typedef struct  {
    int32_t enable;
    int32_t zoom;
} DIGITAL_ZOOM;

typedef struct  {
    float tx_temp;
    float rx_temp;
} TOF_TX_RX_TEMP;

typedef struct  {
    int32_t mode;  //1: 表示单频调制，2: 表示双频调制
    int32_t freq_A;  //第一个调制频率(单频调制只有A频率有效)
    int32_t freq_B;  //第二个调制频率
    int32_t reseved; //保留位
} TOF_MODULATION_FREQ;

typedef struct  {
    int32_t mode;  //1: 表示单频调制，2: 表示双频调制
    float duty_A;  //第一个调制信号占空比， 取值省略百分号，如：54.2
    float duty_B;  //第二个调制信号占空比
    float reseved; //保留位
} TOF_DUTY_CYCLE;

//typedef struct  {
//
//} TOF_CALIBRATION_PARA;

//typedef struct  {
//
//} TOF_DEPTH_COEF_PARA;

//typedef struct  {
//
//} TOF_VCSEL_TEMP_COMPENSATION;

typedef struct  {
    // filter size
    int32_t win_x;
    int32_t win_y;
    float sigma_x;  // x sigma
    float sigma_y;  // y sigma
    uint8_t bypass;    // skip or not； 滤波开关
} TOF_GAUSSIAN_FILTER_PARA;

//typedef struct  {
//
//} TOF_SCATTER_FILTER_PARA;

typedef struct  {
    int32_t 	d;   // filter distance
    float 	sigma_color; // color sigma
    float 	sigma_space;  // space sigma
    uint8_t 	bypass;  // skip or not； 滤波开关
} TOF_BILATERAL_FILTER_PARA;

typedef struct  {
    float 	thres;  // thres for fly point filtering
    float 	noise_coeff;  // noise coeff;
    uint8_t 	fill_hole;   // fill the hole after fly point filtering
    uint8_t 	bypass;  //  skip or not， 滤波开关
} TOF_FLY_POINT_FILTER_PARA;

typedef struct  {
    int32_t 	win_size; // window size
    uint8_t 	bypass; // skip or not； 滤波开关
} TOF_MEDIAN_FILTER_PARA;

typedef struct  {
    float 	confidence_thres;
    uint8_t 	bypass; // skip or not； 滤波开关
} TOF_CONFIDENCE_FILTER_PARA;

typedef struct  {
    int32_t 	nearest_length;   //最近距离， 单位：mm
    int32_t 	farthest_length; //最远距离， 单位：mm
} TOF_NEAREST_FARTHEST_LENGTH;

typedef struct  {
    int32_t upper;  // 阈值上限， 单位：ms
    int32_t lower;  // 阈值下限， 单位：ms
} TOF_EXPOSURE_THRESHOLD_CONTROL;

typedef enum {
    NORMAL,                     //设备状态正常
    WARN,
    FATAL,
} DeviceStateType;

typedef struct  {
    DeviceStateType type;  //设备状态类型
    char msg[256];              //设备状态具体信息
} DEVICE_STATE, ob_device_state;

typedef struct 
{
    float m_SetPointVoltage;
    float m_CompensationVoltage;
    uint16_t m_TecDutyCycle;  // duty cycle on heater/cooler
    uint16_t m_HeatMode;      // TRUE - heat, FALSE - cool
    float m_ProportionalError;
    float m_IntegralError;
    float m_DerivativeError;
    uint16_t m_ScanMode;  // 0 - crude, 1 - precise
} TEC_DATA;

typedef struct 
{
    float fDCmosEmitterDistance;
    float fDCmosRCmosDistance;
    float fReferenceDistance;
    float fReferencePixelSize;
} EXTENSION_PARAMS;

typedef struct 
{
    uint16_t gp_x0;
    uint16_t gp_x1;
    uint16_t gp_x2;
    uint16_t gp_x3;//gp_x0,gp_x1,gp_x2,gp_x3是4条纵向坐标线
    uint16_t gp_y0;
    uint16_t gp_y1;
    uint16_t gp_y2;
    uint16_t gp_y3;//gp_y0,gp_y1,gp_y2,gp_y3是4条横向坐标线
    uint16_t gp_en; //GMP功能开关，1：表示开启，0：表示关闭
    uint16_t gp_conitnuous_zeors_t;//可信全局点 数目为0的连续帧数的阈值
    uint16_t gp_texture_t0;
    uint16_t gp_texture_t1;
    uint16_t gp_ncost_t; //全局计算点的可行度阈值
    uint16_t gp_active_t;//可信点控制阈值，eg：如果设置为4，则满足条件的可信点 数目 大于4应用形变补偿
    uint16_t gp_ref_dy_pos_t; //运算能力 向下 阈值 , eg: 如果 设置为 12, 则向下最多能 矫正 12 行的形变
    uint16_t gp_ref_dy_neg_t;//运算能力向上阈值 , eg: 如果设置为 12, 则向上最多能矫正 12 行的形变
} GPM_CONFIG_DATA;

typedef struct 
{
    uint8_t  dy; //当全局点计算得到的形变量，8bit有符号整数 
    uint16_t ncost; //当前全局点计算的可信度
} GlobalPoint;

typedef struct 
{
    uint8_t gp_ref_dy; //当前gpm单元计算出的用于调整参考图的形变量。
    uint16_t gp_num; //当前有效全局点的数目
    uint16_t gp_continuous_zeros; //可信全局点数目为0的连续帧数
} GlobalPointStatus;

typedef struct 
{
    GlobalPoint gp[16];
    GlobalPointStatus globalPointStatus;
} GPM_STATUS_DATA;

typedef struct 
{
    char ac[48];
} ANTI_COLLUSION_ACTIVATION_CONTENT, ANTI_COLLUSION_ACTIVATION_VERIFY;

#define OB_AUTHORIZATION_CODE_SIZE 16
#define OB_ACTIVATION_CODE_SIZE 49

typedef struct OBAuthorizationCode {
    uint8_t AuthCode[OB_AUTHORIZATION_CODE_SIZE];
}OBAuthorizationCode;

typedef struct OBActivationCode {
    uint8_t ActiveCode[OB_ACTIVATION_CODE_SIZE];
}OBActivationCode;

//IMU采样率值枚举(陀螺仪或加速度计)
typedef enum
{
	OB_IMU_ODR_1_5625_HZ   = 1,
	OB_IMU_ODR_3_125_HZ,
	OB_IMU_ODR_6_25_HZ,
	OB_IMU_ODR_12_5_HZ,
	OB_IMU_ODR_25_HZ,
	OB_IMU_ODR_50_HZ,
	OB_IMU_ODR_100_HZ,
	OB_IMU_ODR_200_HZ,
	OB_IMU_ODR_500_HZ,
	OB_IMU_ODR_1_KHZ,
	OB_IMU_ODR_2_KHZ,
	OB_IMU_ODR_4_KHZ,
	OB_IMU_ODR_8_KHZ,
	OB_IMU_ODR_16_KHZ,
	OB_IMU_ODR_32_KHZ,
}OB_IMU_ODR_EM, OB_GYRO_ODR_EM, OB_ACCEL_ODR_EM;

//陀螺仪量程枚举
typedef enum
{
	OB_GYRO_FS_16dps   = 1,
	OB_GYRO_FS_31dps,
	OB_GYRO_FS_62dps,
	OB_GYRO_FS_125dps,
	OB_GYRO_FS_245dps,
	OB_GYRO_FS_250dps,
	OB_GYRO_FS_500dps,
	OB_GYRO_FS_1000dps,
	OB_GYRO_FS_2000dps,
}OB_GYRO_FS_EM;

//加速度计量程枚举
typedef enum
{
	OB_ACCEL_FS_2g   = 1,
	OB_ACCEL_FS_4g,
	OB_ACCEL_FS_8g,
	OB_ACCEL_FS_16g,
}OB_ACCEL_FS_EM;

//加速度计/陀螺仪支持的采样率列表
typedef struct 
{
    uint32_t num;            //支持的采样率个数(最大不超过16个)
    OB_IMU_ODR_EM odr[16]; //采样率枚举值存放
}IMU_ODR_LIST_PRESET, GYRO_ODR_LIST_PRESET, ACCEL_ODR_LIST_PRESET;		

//陀螺仪支持的量程列表
typedef struct 
{
    uint32_t num;            //支持的量程个数(最大不超过16个)
    OB_GYRO_FS_EM  fs[16]; //量程枚举值存放
}GYRO_FS_LIST_PRESET;	
	
//加速度计支持的量程列表
typedef struct 
{
    uint32_t num;  //支持的量程个数(最大不超过16个)
    OB_ACCEL_FS_EM  fs[16]; //量程枚举值存放
}ACCEL_FS_LIST_PRESET;	
	
    
//IMU数据帧上报结构体
typedef struct 
{
    float fAccData[3];     // 三个方向加速度值(xyz)，单位：g
    float fGyrData[3];     // 三个方向角速度值(xyz)，单位：dps
    uint32_t timestamp;    // 时间戳，单位：us
    uint32_t temp;         // 摄氏度
}IMU_FRAME_DATA;		


//同步模式
typedef enum{
    OB_SYNC_OFF                = 0x00,  //关闭同步
    OB_SYNC_OFFLINE_MODE       = 0x01,  //单机模式  (rgb    --> tof)
    OB_SYNC_ONLINE_HOST_MODE   = 0x02,  //host模式  (rgb    --> tof、ext_out)
    OB_SYNC_ONLINE_SLAVE_MODE  = 0x03,  //slave模式 (ext_in --> rgb、tof、ext_out)
    OB_SYNC_ONLINE_MCU_MODE    = 0x04,    //muc-host模式 (mcu --> rgb、tof、ext_out)
}OB_SYNC_MODE_EM;

typedef struct  {
   uint64_t time;   // sdk->dev: 授时时间; dev->sdk: 设备当前时间
   uint64_t rtt;    // sdk->dev: 命令往返时间，设备接收到后将时间设置为time+rtt/2; dev->sdk: reserve
}DEVICE_TIME;

typedef struct  {
    char numberStr[12];
} DEVICE_SERIAL_NUMBER, DEVICE_PRODUCT_NUMBER;

typedef struct 
{
    float temp_Cal_IR;     // Calibration temperature of IR module.
    float temp_Cal_Laser;  // Calibration temperature of Laser module.
    float ncost_IR;        // Temperature coefficient of IR module.
    float ncost_Laser;     // Temperature coefficient of Laser module.
} TEMP_COMPENSATE_PARA;    

typedef struct 
{
  OB_SYNC_MODE_EM sync_mode; // 0:关闭同步，1：单机模式，2：host模式，3：slave模式, 4: mcu-host
  uint16_t tof_phase_delay;  // us
  uint16_t rgb_phase_delay;  // us
  uint16_t mout_phase_delay; // us
  uint16_t mout_oc_polarity; // 0:正向脉冲, 1: 负向脉冲
  uint16_t mcu_host_fps;     // mcu 主模式时的触发帧率
}TOF_MULTI_DEVICE_SYNC_CONFIG;

typedef enum{
    OB_TOF_FILTER_RANGE_CLOSE= 0,
    OB_TOF_FILTER_RANGE_MIDDLE= 1,
    OB_TOF_FILTER_RANGE_LONG= 2,
    OB_TOF_FILTER_RANGE_DEBUG = 100
}TOF_FILTER_RANGE;
#pragma pack()

#ifdef __cplusplus
}
#endif


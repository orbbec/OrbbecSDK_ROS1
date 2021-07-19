// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

/**
 * @file ObTypes.h
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
#include<stdbool.h>
#include "ObPropertyTypes.h"

typedef struct ob_error    ob_error;

typedef struct ContextImpl ob_context;

typedef struct DeviceImpl ob_device;

typedef struct DeviceInfoImpl ob_device_info;

typedef struct DeviceListImpl ob_device_list;

typedef struct SensorImpl ob_sensor;

//typedef struct ImuSensorImpl ob_imu_sensor;



typedef struct StreamProfileImpl ob_stream_profile;

typedef struct ImuStreamProfileImpl ob_imu_stream_profile;

typedef struct FrameImpl ob_frame;

typedef struct ImuFrameImpl ob_imu_frame;

typedef struct FrameSetImpl ob_frame_set;

typedef struct FilterImpl ob_filter;

typedef struct PipelineImpl ob_pipeline;

typedef struct ConfigImpl ob_config;


typedef void (*ob_device_changed_callback)(ob_device_list* removed, ob_device_list* added, void* user_data);

typedef void (*ob_frame_callback)(ob_frame* frame, void* user_data);

typedef void (*ob_imu_frame_callback)(ob_imu_frame* frame, void* user_data);

typedef void (*ob_frame_set_callback)(ob_frame_set* frame_set, void* user_data);

typedef void (*ob_filter_callback)(ob_frame_set* frame_set, void* user_data);


typedef union
{
    /** XYZ or array representation of vector. */
    struct _xyz
    {
        float x; /**< X component of a vector. */
        float y; /**< Y component of a vector. */
        float z; /**< Z component of a vector. */
    } xyz;       /**< X, Y, Z representation of a vector. */
    float v[3];  /**< Array representation of a vector. */
} float3_t;

typedef struct {
    float rotation[9];
    float translation[3];
} CalibrationExtrinsics;

//=====================================================================Context==============================================================

/**
 * @brief 版本号的描述结构体,例如格式: 0.1.0
 * 
 */
typedef struct {
    /** Major version number, incremented for major API restructuring. */
    int32_t major;
    /** Minor version number, incremented when significant new features added. */
    int32_t minor;
    /** Patch build number, incremented for new releases that primarily provide minor bug fixes. */
    int32_t patch;
} ob_version;

/**
 * @brief 错误码
 * 
 */
typedef enum {
    OB_STATUS_OK                           = 0,
    OB_STATUS_ERROR                        = 1,
    OB_STATUS_NOT_IMPLEMENTED              = 2,
    OB_STATUS_NOT_SUPPORTED                = 3,
    OB_STATUS_BAD_PARAMETER                = 4,
    OB_STATUS_OUT_OF_FLOW                  = 5,
    OB_STATUS_NO_DEVICE                    = 6,
    OB_STATUS_NOT_WRITE_PUBLIC_KEY         = 7,
    OB_STATUS_PUBLIC_KEY_MD5_VERIFY_FAILED = 8,
    OB_STATUS_NOT_WRITE_MD5                = 9,
    OB_STATUS_RSKEY_VERIFY_FAILED          = 10,
    OB_STATUS_NOT_ALIGN                    = 11,
    OB_STATUS_CONTEXT_NOT_INIT             = 12,
    OB_STATUS_TIME_OUT                     = 102,
    OB_STATUS_DEVICE_NOT_FOUND             = 200,
    OB_STATUS_SENSOR_NOT_FOUND             = 300,
} ob_status;

/**
 * @brief log等级
 * 
 */
typedef enum {
    OB_LOG_SEVERITY_DEBUG, /**< Detailed information about ordinary operations */
    OB_LOG_SEVERITY_INFO,  /**< Terse information about ordinary operations */
    OB_LOG_SEVERITY_WARN,  /**< Indication of possible failure */
    OB_LOG_SEVERITY_ERROR, /**< Indication of definite failure */
    OB_LOG_SEVERITY_FATAL, /**< Indication of unrecoverable failure */
    OB_LOG_SEVERITY_NONE,  /**< No logging will occur */
    OB_LOG_SEVERITY_COUNT  /**< Number of enumeration values. Not a valid input:
                              intended to be used in for-loops. */
} ob_log_severity;

/**
 * @brief log类型
 * 
 */
typedef enum {
    OB_LOG_OUT_FILE, /**<log output to files */
    OB_LOG_OUT_TERMINAL,  /**< log output to terminal */
} ob_log_out_type;


/**
 * @brief 异常信息
 * 
 */
typedef enum {
    OB_EXCEPTION_TYPE_UNKNOWN,
    OB_EXCEPTION_TYPE_CAMERA_DISCONNECTED,                                       
    OB_EXCEPTION_TYPE_BACKEND,                                  
    OB_EXCEPTION_TYPE_INVALID_VALUE,           
    OB_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE,                 
    OB_EXCEPTION_TYPE_NOT_IMPLEMENTED,                  
    OB_EXCEPTION_TYPE_DEVICE_IN_RECOVERY_MODE,        
    OB_EXCEPTION_TYPE_IO,                      
    OB_EXCEPTION_TYPE_MEMORY,                  
    OB_EXCEPTION_TYPE_UNSUPPORTED_OPERATION,   
    OB_EXCEPTION_TYPE_COUNT                         
} ob_exception_type;

/**
 * @brief 描述传感器类型的枚举值
 * 
 */
typedef enum {
    OB_SENSOR_UNKNOWN = 0,
    OB_SENSOR_IR      = 1,
    OB_SENSOR_COLOR   = 2,
    OB_SENSOR_DEPTH   = 3,
    OB_SENSOR_IMU     = 4,
} ob_sensor_type;

/**
 * @brief pipeline的输出旋转
 *
 */
typedef enum {
	OB_ORIENTATION_PORTRAIT  = 0,
	OB_ORIENTATION_LANDSCAPE = 1,
} ob_orientation_type;


typedef union
{
    int32_t intValue;
    float floatValue;
} OBPropertyValue;

typedef double ob_time; /**< Timestamp format. units are milliseconds */

//====================================================================================Stream & Frame  type =============================================

/**
 * @brief 描述数据流类型的枚举值
 * 
 */
typedef enum {
    OB_STREAM_VIDEO = 0,
    OB_STREAM_IR    = 1,
    OB_STREAM_COLOR = 2,
    OB_STREAM_DEPTH = 3,
    OB_STREAM_IMU   = 4,
} ob_stream_type;

/**
 * @brief 描述Frame类型枚举值
 * 
 */
typedef enum {
    OB_FRAME_VIDEO  = 0,
    OB_FRAME_IR     = 1,
    OB_FRAME_COLOR  = 2,
    OB_FRAME_DEPTH  = 3,
    OB_FRAME_IMU    = 4,
    OB_FRAME_ASSIST = 5,
    OB_FRAME_POINTS = 6,
} ob_frame_type;


/**
 * @brief 数据帧的元数据类型
 * 
 */
typedef enum {
    OB_FRAME_METADATA_FRAME_COUNTER,    
    OB_FRAME_METADATA_FRAME_TIMESTAMP,  
    OB_FRAME_METADATA_SENSOR_TIMESTAMP, 
    OB_FRAME_METADATA_ACTUAL_EXPOSURE,  
    OB_FRAME_METADATA_GAIN_LEVEL,
} ob_frame_metadata_type;


/**
 * @brief 描述像素格式的枚举值
 * 
 */
typedef enum {
    OB_FORMAT_YUYV = 0,
    OB_FORMAT_YUY2,
    OB_FORMAT_UYVY,
    OB_FORMAT_NV12,
    OB_FORMAT_NV21,
    OB_FORMAT_MJPG,
    OB_FORMAT_H264,
    OB_FORMAT_H265,
    OB_FORMAT_Y16,
    OB_FORMAT_Y8,
    OB_FORMAT_Y10,
    OB_FORMAT_Y11,
    OB_FORMAT_Y12,
    OB_FORMAT_GRAY,
    OB_FORMAT_HEVC,
    OB_FORMAT_I420,
    OB_FORMAT_ACCEL,
    OB_FORMAT_GYRO,
    OB_FORMAT_IMU,
    OB_FORMAT_POINT,
    OB_FORMAT_RGB_POINT,
    OB_FORMAT_UNKNOWN,
} ob_format;

typedef struct BaselineCalibrationPara {
    float fZ0;
    float fBaseline;
} BaselineCalibrationPara;

typedef struct MultipleDistanceCalibrationPara {
    uint32_t       nSize;
    uint8_t* data;
} MultipleDistanceCalibrationPara;

//======================================================================Pipeline type ==================================================
/**
 * @brief SDK中3D点描述
 * 
 */
typedef struct {
    float x;
    float y;
    float z;
} ob_point;

/**
 * @brief 带有颜色信息的3D点描述
 * 
 */
typedef struct {
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;
} ob_color_point;
//typedef enum 
//{
//    OB_POINT_DEPTH = 0,  // point data type: ob_point
//    OB_POINT_RGBD  = 1   // point data type: ob_color_point
//} ob_point_type;

/**
 * @brief 对齐的输入参数
 * 
 */
typedef struct {
   // OBVideoMode depthVideo;
    //OBVideoMode colorVideo;
    int32_t         softAlign;  // 1=>software Align, 0=>hardware Align
    int32_t         HwAlignOn;
} ob_align_parameter;

/**
 * @brief 在Pipline高层接口的配置流的参数信息
 * 
 */
typedef struct {
    int32_t      Enable;  // 0->false ,1->true
    ob_format obFormat_;
    int32_t      width;
    int32_t      height;
    int32_t      fps;

} ob_stream_profile_config;

typedef struct ConfigImpl OBConfig;

/**
 * @brief 对齐参数
typedef struct OBSensorInfo OBSensorInfo;
 * 
 */
typedef enum {
    OB_ALIGN_ENABLE,
    OB_ALIGN_DISABLE,
} ob_align_status;

typedef struct OBSensorInfo OBSensorInfo;

typedef struct OBFrameSet OBFrameSet;

/**
 * @brief 软件滤波参数
 * 
 */
typedef struct {
	uint32_t maxSpeckleSize;
	uint32_t maxDiff;
	uint32_t filterType;  //0 stands for disparity 1 stands  for depth .default filterType=1
	uint32_t isEnable;  //0 stands for disable, 1 stands for enable
} ob_soft_filter_parameter;

/**
 * @brief 双目视差转深度所需参数
 * 
 */
typedef struct {
    double fx;
    double baseline;
    double fCoefficient;
    int32_t    nShiftScale;
    double disparityCoeff;
    int32_t    maxDepthValue;
    int32_t    minDepthValue;
} ob_dual_disparity_process_parameter;

/**
 * @brief 单目视差转深度所需参数
 * 
 */
typedef struct {
    double fZeroPlaneDistance;  // Z0
    double nParamCoeff;
    double fZeroPlanePixelSize;
    float  fEmitterDComsDistance;  // baseline
    double nShiftScale;
    int32_t    nConstShift;
    int32_t    maxDepthValue;
    int32_t    minDepthValue;
    int32_t    nPixelSizeFactor;
} ob_single_disparity_process_parameter;

/**
 * @brief 视差转深度所需参数
 * 
 */
typedef union {
    ob_dual_disparity_process_parameter   dualParameter;
    ob_single_disparity_process_parameter singleParameter;
    int32_t isEnable;  //0 stands for disable, 1 stands for enable
} ob_disparity_process_parameter;


/**
 * @brief 深度处理所需的相关参数
 * 
 */
typedef struct {
    ob_disparity_process_parameter disparityParameter;
    ob_soft_filter_parameter       softFilterParameter;
    int32_t                         isDualCamera;
} ob_depth_process_parameter;

typedef struct {
    uint16_t nErrorCode;
} file_result;

typedef enum {
    FILE_PORT_FIRMWARE_UPGRADE = 0,
    FILE_PORT_CALIBRATION_UPGRADE = 1,
    FILE_PORT_FILE_TURN = 2,
} file_port;

typedef struct UpgradeStatus {
    int16_t    state;
    int16_t    percentage;
    uint8_t  message[256];
} UpgradeStatus;

typedef enum {
    STAT_FILE_TRANSFER = 4,
    STAT_DONE = 3,
    STAT_IN_PROGRESS = 2,
    STAT_START = 1,
    STAT_VERIFY_IMAGE = 0,
    ERR_VERIFY = -1,
    ERR_PROGRAM = -2,
    ERR_ERASE = -3,
    ERR_FLASH_TYPE = -4,
    ERR_IMG_SIZE = -5,
    ERR_OTHER = -6,
    ERR_DDR = -7,
    ERR_TIMEOUT = -8
} UpgradeState;

typedef enum {
    FILE_TRAN_STAT_TRANSFER     = 2,
    FILE_TRAN_STAT_DONE         = 1,
    FILE_TRAN_STAT_PREPAR       = 0,
    FILE_TRAN_ERR_DDR           = -1,
    FILE_TRAN_ERR_NOT_ENOUGH_SPACE  = -2,
    FILE_TRAN_ERR_PATH_NOT_WRITABLE = -3,
    FILE_TRAN_ERR_MD5_ERROR         = -4,
    FILE_TRAN_ERR_WRITA_FLASH_ERROR = -5,
    FILE_TRAN_ERR_TIMEOUT = -6
} FileTranState;

typedef enum{
    DATA_TRAN_STAT_STOPED = 3,
    DATA_TRAN_STAT_DONE = 2,
    DATA_TRAN_STAT_VERIFYING = 1,
    DATA_TRAN_STAT_TRANSFERING = 0,
    DATA_TRAN_ERR_BUSY = -1,
    DATA_TRAN_ERR_UNSUPPORTED = -2,
    DATA_TRAN_ERR_TRAN_FAILED = -3,
    DATA_TRAN_ERR_VERIFY_FAILED = -4,
    DATA_TRAN_ERR_OTHER = -5
} DataTranState;

typedef struct 
{
    void* data;             // 当前数据块数据
    uint32_t size;          // 当前数据块大小
    uint32_t offset;        // 当前数据块相对完整数据的偏移
    uint32_t fullDataSize;  // 完整数据大小
} DataChunk;

typedef enum { 
    DEPTH_SOFT_FILTER_OFF = 0, 
    DEPTH_SOFT_FILTER_DEVICE_SIDE, 
    DEPTH_SOFT_FILTER_SDK_SIDE 
} DepthSoftFilterControl;

typedef struct SDKVersion {
	uint8_t  nMajor;  //  主版本号
	uint8_t  nMinor;  // 次版本号
	uint16_t nBuild;  //修订版本号
} SDKVersion;

typedef struct
{
    OBPropertyValue cur;
    OBPropertyValue max;
    OBPropertyValue min;
    OBPropertyValue step;
    OBPropertyValue def;
} OBPropertyRange;

//整形范围的结构体
typedef struct
{
    int32_t cur;
    int32_t max;
    int32_t min;
    int32_t step;
    int32_t def;
} OBIntPropertyRange, ob_int_property_range;

//浮点型范围的结构体
typedef struct
{
    float cur;
    float max;
    float min;
    float step;
    float def;
} OBFloatPropertyRange, ob_float_property_range;

// bool型范围的结构体
typedef struct
{
    bool cur;
    bool max;
    bool min;
    bool step;
    bool def;
} OBBoolPropertyRange, ob_bool_property_range;

// 相机内参
typedef struct
{
    float fx; /// x方向焦距，单位：像素
    float fy; /// y方向焦距，单位：像素
    float cx; /// 光心横坐标
    float cy; /// 光心纵坐标
    int16_t width; /// 图像宽度
    int16_t height; /// 图像高度
}OBCameraIntrinsic, ob_camera_intrinsic; 
// 去畸变参数
typedef struct
{
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    float k6;
    float p1;
    float p2;
}OBCameraDistortion, ob_camera_distortion;

// 旋转矩阵
typedef struct
{
    float rot[9]; // 旋转矩阵，行优先
    float trans[3];
}OBD2CTransform, ob_d2c_transform;


typedef void ( *ob_file_send_callback )( FileTranState state, const char* message, uint8_t percent, void* user_data );
typedef void ( *ob_device_upgrade_callback )( UpgradeState state, const char* message, uint8_t percent, void* user_data );
typedef void ( *ob_device_state_callback )( DEVICE_STATE state, void* user_data );
typedef void ( *ob_set_data_callback)(DataTranState state, uint8_t percent, void* user_data );
typedef void ( *ob_get_data_callback)(DataTranState state, DataChunk *dataChunk, void* user_data );

#ifdef __cplusplus
    }
#endif

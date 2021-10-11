// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

/**
 * @file InternalTypes.h
 * @brief 提供SDK的结构体、枚举常量定义
 */

#pragma once

#if ( defined( WIN32 ) || defined( _WIN32 ) || defined( WINCE ) )
#ifdef OB_EXPORTS
#define OB_EXTENSION_API __declspec( dllexport )
#else
#define OB_EXTENSION_API __declspec( dllimport )
#endif
#else
#define OB_EXTENSION_API
#endif

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#pragma pack( push, 1 )  // struct 1-byte align

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

/**========================================================================以下类型是供工程软件使用 =============================================
 *
 *  以下类型为SDK内部及工程模式内部使用，不对外使用。对开发者正式使用的SDK的版本，使用以下接口无效或抛出异常！！！！！！
 *
 * *============================================================================================== =============================================
 */

typedef union {
    struct _xyz {
        float x;
        float y;
        float z;
    } xyz;
    float v[ 3 ];
} float3_t;

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

typedef struct {
    float rotation[ 9 ];
    float translation[ 3 ];
} CalibrationExtrinsics;

typedef union {
    int32_t intValue;
    float   floatValue;
} OBPropertyValue;

typedef double ob_time;

typedef struct {
    OBPropertyValue cur;
    OBPropertyValue max;
    OBPropertyValue min;
    OBPropertyValue step;
    OBPropertyValue def;
} OBPropertyRange;

typedef struct {
    char    firmwareVersion[ 16 ];   // 如：1.2.18
    char    hardwareVersion[ 16 ];   // 如：1.0.18
    char    sdkVersion[ 16 ];        // 支持最低sdk的版本号，sdk版本号：2.3.2（主.次.修订)
    char    depthChip[ 16 ];         // mx6000, mx6600
    char    systemChip[ 16 ];        // ar9201
    char    serialNumber[ 16 ];      // 序列号
    int32_t deviceType;              // 单目，双目，tof。枚举值
    char    deviceName[ 16 ];        // 设备名称 astra+
    char    subSystemVersion[ 16 ];  // 如：倚天剑的MCU固件版本 1.0.23
    char    resever[ 32 ];           // 保留
} VERSIONS, ob_versions_info;

/**
 * @brief 对齐参数
 */
typedef enum {
    OB_ALIGN_ENABLE,
    OB_ALIGN_DISABLE,
} ob_align_status;

typedef enum {
    FILE_PORT_FIRMWARE_UPGRADE    = 0,
    FILE_PORT_CALIBRATION_UPGRADE = 1,
    FILE_PORT_FILE_TURN           = 2,
} file_port;

typedef struct UpgradeStatus {
    int16_t state;
    int16_t percentage;
    uint8_t message[ 256 ];
} UpgradeStatus;

//获取浮点型数据传输结构体
typedef struct {
    float cur;
    float max;
    float min;
    float step;
    float def;
} OBFloatPropertyRangeTran, ob_float_property_range_tran;
// 整形范围数据传输结构体
typedef struct {
    int32_t cur;
    int32_t max;
    int32_t min;
    int32_t step;
    int32_t def;
} OBIntPropertyRangeTran, ob_int_property_range_tran;

// bool型范围数据传输结构体 (根据SDK与设备porperty类型数据通信协议，
// 必须保证结构体内每个字段都为4字节, 所以这里将各字段类型定义为uint32_t,
// uint32_t与bool类型转换遵循C++/C 语言标准规则)
typedef struct {
    uint32_t cur;
    uint32_t max;
    uint32_t min;
    uint32_t step;
    uint32_t def;
} OBBoolPropertyRangeTran, ob_bool_property_range_tran;

typedef struct {
    float    d_intr_p[ 4 ];    // 深度相机内参：[fx,fy,cx,cy]
    float    c_intr_p[ 4 ];    // 彩色相机内参：[fx,fy,cx,cy]
    float    d2c_r[ 9 ];       // 深度相机到彩色相机的旋转矩阵 [r00,r01,r02;r10,r11,r12;r20,r21,r22]
    float    d2c_t[ 3 ];       // 深度相机到彩色相机的平移矩阵 [t1,t2,t3]
    float    d_k[ 5 ];         // 深度相机畸变参数 [k1,k2,p1,p2,k3]  // 注意k3的位置，这个是算法定的，有些代码k3排在k2后边
    float    c_k[ 5 ];         // 彩色相机畸变参数 [k1,k2,p1,p2,k3]
    uint32_t c_img_size[ 2 ];  // 彩色标定分辨率 [color_width, color_height]
    uint32_t d_img_size[ 2 ];  // 深度标定分辨率 [ir_width, ir_height]

} CAMERA_PARA, ob_camera_para;

/**
 * @brief 软件滤波参数
 *
 */
typedef struct {
    uint32_t maxSpeckleSize;
    uint32_t maxDiff;
    uint32_t filterType;  // 0 表示对视差滤波， 1 表示对深度滤波 ，默认值是1
    uint32_t isEnable;    // 0 表示关闭, 1 表示开启
} ob_soft_filter_parameter;

/**
 * @brief 双目视差转深度所需参数
 *
 */
typedef struct {
    double  fx;
    double  baseline;
    double  fCoefficient;
    int32_t nShiftScale;
    double  disparityCoeff;
    int32_t maxDepthValue;
    int32_t minDepthValue;
} ob_dual_disparity_process_parameter;

/**
 * @brief 单目视差转深度所需参数
 *
 */
typedef struct {
    double  fZeroPlaneDistance;  // 表示标定平面Z0
    double  nParamCoeff;
    double  fZeroPlanePixelSize;
    float   fEmitterDComsDistance;  // 标定的基线：baseline
    double  nShiftScale;
    int32_t nConstShift;
    int32_t maxDepthValue;
    int32_t minDepthValue;
    int32_t nPixelSizeFactor;
} ob_single_disparity_process_parameter;

/**
 * @brief 视差转深度所需参数
 *
 */
typedef union {
    ob_dual_disparity_process_parameter   dualParameter;
    ob_single_disparity_process_parameter singleParameter;
    int32_t                               isEnable;  // 0代表关闭, 1 表示开启
} ob_disparity_process_parameter;

/**
 * @brief 深度处理所需的相关参数
 *
 */
typedef struct {
    ob_disparity_process_parameter disparityParameter;
    ob_soft_filter_parameter       softFilterParameter;
    int32_t                        isDualCamera;
    bool                           rleDecodeEnable;
} ob_depth_process_parameter;

typedef struct {
    float fBaseline;
    float fZ0;
} BASELINE_CALIBRATION_PARA, ob_baseline_calibration_para;

typedef struct {
    float cpuTemp;
    float irTemp;
    float ldmTemp;
    float mainBoardTemp;
    float tecTemp;
    float imuTemp;
} DEVICE_TEMPERATURE, ob_device_temperature, OBDeviceTemperature;

typedef struct {
    uint32_t min;
    uint32_t max;
} RANGE, DEPHT_VALUE_LIMIT_RANGE;

typedef struct {
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
} DEVICE_AE_PARAMS, ob_device_ae_params;

typedef struct {
    float fDCmosEmitterDistance;
    float fDCmosRCmosDistance;
    float fReferenceDistance;
    float fReferencePixelSize;
} EXTENSION_PARAMS, ob_extension_params;

typedef struct {
    int16_t state;
    int16_t percentage;
    uint8_t message[ 256 ];
} DEVICE_UPGRADE_STATUS, ob_device_upgrade_status;

typedef struct {
    float tx_temp;
    float rx_temp;
} TOF_TX_RX_TEMP, ob_tof_tx_rx_temp;

typedef struct {
    int32_t mode;     // 1: 表示单频调制，2: 表示双频调制
    int32_t freq_A;   //第一个调制频率(单频调制只有A频率有效)
    int32_t freq_B;   //第二个调制频率
    int32_t reseved;  //保留位
} TOF_MODULATION_FREQ, ob_tof_modulation_freq;

typedef struct {
    int32_t mode;     // 1: 表示单频调制，2: 表示双频调制
    float   duty_A;   //第一个调制信号占空比， 取值省略百分号，如：54.2
    float   duty_B;   //第二个调制信号占空比
    float   reseved;  //保留位
} TOF_DUTY_CYCLE, ob_tof_duty_cycle;

typedef struct {
    // filter size
    int32_t win_x;
    int32_t win_y;
    float   sigma_x;  // x sigma
    float   sigma_y;  // y sigma
    uint8_t bypass;   // skip or not； 滤波开关
} TOF_GAUSSIAN_FILTER_PARA, ob_tof_gaussian_filter_para;

typedef struct {
    int32_t d;            // filter distance
    float   sigma_color;  // color sigma
    float   sigma_space;  // space sigma
    uint8_t bypass;       // skip or not； 滤波开关
} TOF_BILATERAL_FILTER_PARA, ob_tof_bilateral_filter_para;

typedef struct {
    float   thres;        // thres for fly point filtering
    float   noise_coeff;  // noise coeff;
    uint8_t fill_hole;    // fill the hole after fly point filtering
    uint8_t bypass;       //  skip or not， 滤波开关
} TOF_FLY_POINT_FILTER_PARA, ob_tof_fly_point_filter_para;

typedef struct {
    int32_t win_size;  // window size
    uint8_t bypass;    // skip or not； 滤波开关
} TOF_MEDIAN_FILTER_PARA, ob_tof_median_filter_para;

typedef struct {
    float   confidence_thres;
    uint8_t bypass;  // skip or not； 滤波开关
} TOF_CONFIDENCE_FILTER_PARA, ob_tof_confidence_filter_para;

typedef struct {
    int32_t nearest_length;   //最近距离， 单位：mm
    int32_t farthest_length;  //最远距离， 单位：mm
} TOF_NEAREST_FARTHEST_LENGTH, ob_tof_nearest_farthest_length;

typedef struct {
    int32_t upper;  // 阈值上限， 单位：ms
    int32_t lower;  // 阈值下限， 单位：ms
} TOF_EXPOSURE_THRESHOLD_CONTROL, ob_tof_exposure_threshold_control;

typedef enum {
    NORMAL                      = 0,
    OPEN_STREAM_OPERATION_ERROR = 1,  //开流异常
} DeviceStateType;

typedef struct {
    DeviceStateType type;        //设备状态类型
    char            msg[ 256 ];  //设备状态具体信息
} DEVICE_STATE, ob_device_state, OBDeviceState;

typedef struct {
    float    m_SetPointVoltage;
    float    m_CompensationVoltage;
    uint16_t m_TecDutyCycle;  // duty cycle on heater/cooler
    uint16_t m_HeatMode;      // TRUE - heat, FALSE - cool
    float    m_ProportionalError;
    float    m_IntegralError;
    float    m_DerivativeError;
    uint16_t m_ScanMode;  // 0 - crude, 1 - precise
    float    m_Pvar;
    float    m_Ivar;
    float    m_Dvar;
} TEC_DATA, ob_tec_data;

// IMU采样率值枚举(陀螺仪或加速度计)
typedef enum {
    OB_IMU_ODR_1_5625_HZ = 1,
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
} OB_IMU_ODR_EM,
    OB_GYRO_ODR_EM, OB_ACCEL_ODR_EM;

//陀螺仪量程枚举
typedef enum {
    OB_GYRO_FS_16dps = 1,
    OB_GYRO_FS_31dps,
    OB_GYRO_FS_62dps,
    OB_GYRO_FS_125dps,
    OB_GYRO_FS_245dps,
    OB_GYRO_FS_250dps,
    OB_GYRO_FS_500dps,
    OB_GYRO_FS_1000dps,
    OB_GYRO_FS_2000dps,
} OB_GYRO_FS_EM;

//加速度计量程枚举
typedef enum {
    OB_ACCEL_FS_2g = 1,
    OB_ACCEL_FS_4g,
    OB_ACCEL_FS_8g,
    OB_ACCEL_FS_16g,
} OB_ACCEL_FS_EM;

//深度单位枚举
typedef enum {
    OB_DEPTH_UNIT_MM_0P1,
    OB_DEPTH_UNIT_MM_0P2,
    OB_DEPTH_UNIT_MM_0P25,
    OB_DEPTH_UNIT_MM_0P4,
    OB_DEPTH_UNIT_MM_0P8,
    OB_DEPTH_UNIT_MM_1P6,
} OB_DEPTH_UNIT_EM;

//加速度计/陀螺仪支持的采样率列表
typedef struct {
    uint32_t      num;        //支持的采样率个数(最大不超过16个)
    OB_IMU_ODR_EM odr[ 16 ];  //采样率枚举值存放
} IMU_ODR_LIST_PRESET, GYRO_ODR_LIST_PRESET, ACCEL_ODR_LIST_PRESET;

//陀螺仪支持的量程列表
typedef struct {
    uint32_t      num;       //支持的量程个数(最大不超过16个)
    OB_GYRO_FS_EM fs[ 16 ];  //量程枚举值存放
} GYRO_FS_LIST_PRESET;

//加速度计支持的量程列表
typedef struct {
    uint32_t       num;       //支持的量程个数(最大不超过16个)
    OB_ACCEL_FS_EM fs[ 16 ];  //量程枚举值存放
} ACCEL_FS_LIST_PRESET;

// IMU数据帧上报结构体
typedef struct {
    float    fAccData[ 3 ];  // 三个方向加速度值(xyz)，单位：g
    float    fGyrData[ 3 ];  // 三个方向角速度值(xyz)，单位：dps
    uint32_t timestamp;      // 时间戳，单位：us
    uint32_t temp;           // 摄氏度
} IMU_FRAME_DATA;

typedef struct {
    uint64_t time;  // sdk->dev: 授时时间; dev->sdk: 设备当前时间
    uint64_t rtt;   // sdk->dev: 命令往返时间，设备接收到后将时间设置为time+rtt/2; dev->sdk: reserve
} DEVICE_TIME, ob_device_time;

typedef struct {
    char numberStr[ 12 ];
} DEVICE_SERIAL_NUMBER, DEVICE_PRODUCT_NUMBER, ob_device_serial_number, ob_device_product_number;

//同步模式
typedef enum {
    OB_SYNC_OFF               = 0x00,  //关闭同步
    OB_SYNC_OFFLINE_MODE      = 0x01,  //单机模式  (rgb    --> tof)
    OB_SYNC_ONLINE_HOST_MODE  = 0x02,  // host模式  (rgb    --> tof、ext_out)
    OB_SYNC_ONLINE_SLAVE_MODE = 0x03,  // slave模式 (ext_in --> rgb、tof、ext_out)
    OB_SYNC_ONLINE_MCU_MODE   = 0x04,  // muc-host模式 (mcu --> rgb、tof、ext_out)
} OB_SYNC_MODE_EM;

typedef struct {
    OB_SYNC_MODE_EM sync_mode;         // 0:关闭同步，1：单机模式，2：host模式，3：slave模式, 4: mcu-host
    uint16_t        tof_phase_delay;   // us
    uint16_t        rgb_phase_delay;   // us
    uint16_t        mout_phase_delay;  // us
    uint16_t        mout_oc_polarity;  // 0:正向脉冲, 1: 负向脉冲
    uint16_t        mcu_host_fps;      // mcu 主模式时的触发帧率
} TOF_MULTI_DEVICE_SYNC_CONFIG, ob_tof_multi_device_sync_config;

typedef struct {
    float temp_Cal_IR;     // Calibration temperature of IR module.
    float temp_Cal_Laser;  // Calibration temperature of Laser module.
    float ncost_IR;        // Temperature coefficient of IR module.
    float ncost_Laser;     // Temperature coefficient of Laser module.
} TEMP_COMPENSATE_PARA, ob_temp_compensate_para;

#define OB_ACTIVATION_CODE_SIZE 49
typedef struct {
    uint8_t ActiveCode[ OB_ACTIVATION_CODE_SIZE ];
} OBActivationCode, ob_activation_code;

typedef enum { OB_TOF_FILTER_RANGE_CLOSE = 0, OB_TOF_FILTER_RANGE_MIDDLE = 1, OB_TOF_FILTER_RANGE_LONG = 2, OB_TOF_FILTER_RANGE_DEBUG = 100 } TOF_FILTER_RANGE;
#pragma pack( pop )

#ifdef __cplusplus
}
#endif

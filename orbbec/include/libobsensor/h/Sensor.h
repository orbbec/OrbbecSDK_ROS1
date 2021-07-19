/**
 * @file Sensor.h
 * @brief 传感器相关函数，用于获取流配置，开关流，设置及获取传感器属性等操作
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 获取传感器类型
 * 
 * @param[in] sensor 传感器对象
 * @param[out] error 记录错误信息
 * @return ob_sensor_type 返回传感器类型
 */
ob_sensor_type ob_sensor_get_type(ob_sensor *sensor, ob_error **error);

/*
 * @brief 设置bool类型的设备属性
 *
 * @param[in] sensor 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 */
void ob_sensor_set_bool_property(ob_sensor* sensor, ob_global_unified_property property_id, bool property,
    ob_error** error);

/**
 * @brief 获取bool类型的设备属性
 *
 * @param[in] sensor 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return bool 返回属性值
 */
bool ob_sensor_get_bool_property(ob_sensor* sensor, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief 设置int类型的设备属性
 *
 * @param[in] sensor 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 */
void ob_sensor_set_int_property(ob_sensor* sensor, ob_global_unified_property property_id, int32_t property,
    ob_error** error);

/**
 * @brief 获取int类型的设备属性
 *
 * @param[in] sensor 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return int32_t 返回属性值
 */
int32_t ob_sensor_get_int_property(ob_sensor* sensor, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief 设置float类型的设备属性
 *
 * @param[in] sensor 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 */
void ob_sensor_set_float_property(ob_sensor* sensor, ob_global_unified_property property_id, float property,
    ob_error** error);

/**
 * @brief 获取float类型的设备属性
 *
 * @param[in] sensor 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return int32_t 返回属性值
 */
float ob_sensor_get_float_property(ob_sensor* sensor, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief 设置结构体类型的设备属性
 * 
 * @param[in] sensor 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] data 要设置的属性数据
 * @param[in] data_size 要设置的属性大小
 * @param[out] error 记录错误信息
 */
void ob_sensor_set_structured_data(ob_sensor *sensor, ob_global_unified_property property_id, const void *data,
    uint32_t data_size, ob_error **error);

/**
 * @brief 获取结构体类型的设备属性
 * 
 * @param[in] sensor 设备对象
 * @param[in] property_id 要获取的属性id
 * @param[out] data 获取的属性数据
 * @param[out] data_size 获取的属性大小
 * @param[out] error 记录错误信息
 */
void ob_sensor_get_structured_data(ob_sensor *sensor, ob_global_unified_property property_id, void *data,
    uint32_t *data_size, ob_error **error);

/**
 * @brief 获取raw_data类型的传感器属性
 * 
 * @param[in] sensor 传感器对象
 * @param[in] property_id 属性的id
 * @param[in] cb 获取数据及进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回 
 * @param[out] error 记录错误信息
 */
bool ob_sensor_get_raw_data(
    ob_sensor *sensor, ob_global_unified_property property_id, 
    ob_get_data_callback cb, bool async,
    void *user_data, ob_error **error
    );

/**
 * @brief 设置raw data类型的传感器属性
 * 
 * @param[in] sensor 传感器对象
 * @param[in] property_id 属性的id
 * @param[in] data 要设置的属性数据
 * @param[in] data_size 设置的属性大小
 * @param[in] cb 进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回 
 * @param[out] error 记录错误信息
 */
bool ob_sensor_set_raw_data(
    ob_sensor *sensor, ob_global_unified_property property_id, 
    void *data, uint32_t data_size, 
    ob_set_data_callback cb, bool async,
    void *user_data, ob_error **error
);

/**
 * @brief 获取int类型传感器属性的范围
 * 
 * @param[in] sensor 传感器对象
 * @param[in] property_id 属性的id
 * @param[out] error 记录错误信息
 * @return ob_int_property_range 返回传感器属性范围
 */
ob_int_property_range
ob_sensor_get_int_property_range(ob_sensor *sensor, ob_global_unified_property property_id, ob_error **error);

/**
 * @brief 获取float类型传感器属性的范围
 *
 * @param[in] sensor 传感器对象
 * @param[in] property_id 属性的id
 * @param[out] error 记录错误信息
 * @return ob_int_property_range 返回传感器属性范围
 */
ob_float_property_range
ob_sensor_get_float_property_range(ob_sensor* sensor, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief 获取bool类型传感器属性的范围
 *
 * @param[in] sensor 传感器对象
 * @param[in] property_id 属性的id
 * @param[out] error 记录错误信息
 * @return ob_bool_property_range 返回传感器属性范围
 */
ob_bool_property_range
ob_sensor_get_bool_property_range(ob_sensor* sensor, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief 查询传感器属性是否支持
 * 
 * @param[in] sensor 传感器对象
 * @param[in] property_id 要查询的属性id
 * @param[out] error 记录错误信息
 * @return bool 返回属性是否支持
 */
bool
ob_sensor_is_property_supported(ob_sensor *sensor, ob_global_unified_property property_id, ob_error **error);

/**
 * @brief 获取传感器支持的所有流的配置
 * 
 * @param[in] sensor 传感器对象
 * @param[out] count 获取的流配置的数量
 * @param[out] error 记录错误信息
 * @return ob_stream_profile** 返回流配置的列表
 */
ob_stream_profile **
ob_sensor_get_stream_profiles(ob_sensor *sensor, uint32_t *count, ob_error **error);

/**
 * @brief 打开传感器的流，并设置帧数据回调
 * 
 * @param[in] sensor 传感器对象
 * @param[in] profile 流的配置信息
 * @param[in] callback 帧数据到达时触发的回调函数
 * @param[in] user_data 可以传入任意用户数据，并从回调中获取
 * @param[out] error 记录错误信息
 */
void ob_sensor_start(ob_sensor *sensor, ob_stream_profile *profile, ob_frame_callback callback,
                     void *user_data, ob_error **error);

/**
 * @brief 停止传感器的流
 * 
 * @param[in] sensor 传感器对象
 * @param[out] error 记录错误信息
 */
void ob_sensor_stop(ob_sensor *sensor, ob_error **error);

/**
 * @brief 切换传感器的流配置
 *
 * @param[in] sensor 传感器对象
 * @param[in] profile 流的配置
 * @param[out] error 记录错误信息
 */
void ob_sensor_switch_profile(ob_sensor* sensor, ob_stream_profile* profile, ob_error** error);

/**
 * @brief 删除传感器对象列表
 * 
 * @param[in] sensors 要删除的传感器对象列表
 * @param[in] count 传感器数量
 * @param[out] error 记录错误信息
 */
void ob_delete_sensors(ob_sensor **sensors, uint32_t count, ob_error **error);

/**
 * @brief 删除传感器对象
 * 
 * @param[in] sensor 要删除的传感器对象
 * @param[out] error 记录错误信息
 */
void ob_delete_sensor(ob_sensor *sensor, ob_error **error);

/**
 * @brief 获取imu传感器支持的所有流的配置
 * 
 * @param[in] sensor 传感器对象
 * @param[out] count 获取的流配置的数量
 * @param[out] error 记录错误信息
 * @return ob_imu_stream_profile** 返回流配置的列表
 */
ob_imu_stream_profile** ob_sensor_get_imu_stream_profiles(ob_sensor* sensor, uint32_t* count, ob_error **error);

/**
 * @brief 打开imu传感器的流，并设置imu帧数据回调
 * 
 * @param[in] sensor 传感器对象
 * @param[in] profile 流的配置信息
 * @param[in] callback 帧数据到达时触发的回调函数
 * @param[in] user_data 可以传入任意用户数据，并从回调中获取
 * @param[out] error 记录错误信息
 */
void ob_sensor_imu_start(ob_sensor* sensor, ob_imu_stream_profile* profile, ob_imu_frame_callback callback,
                     void *user_data, ob_error **error);

/**
 * @brief 停止imu传感器的流
 * 
 * @param[in] sensor 传感器对象
 * @param[out] error 记录错误信息
 */
void ob_sensor_imu_stop(ob_sensor* sensor, ob_error **error);

#ifdef __cplusplus
}
#endif
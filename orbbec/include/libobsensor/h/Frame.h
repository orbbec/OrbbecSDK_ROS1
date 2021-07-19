/**
 * @file Frame.h
 * @brief 帧相关函数，主要用于获取帧数据及帧的信息
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 获取帧索引
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧索引
 */
uint64_t ob_frame_index(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧宽
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧宽
 */
uint32_t ob_frame_width(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧高
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧高
 */
uint32_t ob_frame_height(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧格式
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_format 返回帧格式
 */
ob_format ob_frame_format(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧类型
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_frame_type 返回帧类型
 */
ob_frame_type ob_frame_get_type(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧硬件时间戳
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧硬件时间戳
 */
uint64_t ob_frame_time_stamp(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧系统时间戳
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧系统时间戳
 */
uint64_t ob_frame_system_time_stamp(ob_frame *frame, ob_error **error);

/**
 * @brief 获取imu温度
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return float imu温度
 */
float ob_imu_frame_temperature(ob_imu_frame *frame, ob_error **error);

/**
 * @brief 获取imu加速度数据
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return float3_t 返回imu加速度数据
 */
float3_t ob_imu_frame_accel_data(ob_imu_frame *frame, ob_error **error);

/**
 * @brief 获取imu陀螺仪数据
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return float3_t 返回imu陀螺仪数据
 */
float3_t ob_imu_frame_gyro_data(ob_imu_frame *frame, ob_error **error);

/**
 * @brief 获取imu帧时间戳
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回imu帧时间戳
 */
uint64_t ob_imu_frame_time_stamp(ob_imu_frame *frame, ob_error **error);

/**
 * @brief 获取imu帧索引
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint64_t 返回帧索引
 */
uint64_t ob_imu_frame_index(ob_imu_frame *frame, ob_error **error);

/**
 * @brief 获取imu帧类型
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_frame_type 返回帧类型
 */
ob_frame_type ob_imu_frame_get_type(ob_imu_frame *frame, ob_error **error);

/**
 * @brief 获取imu帧的流类型
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_stream_type 返回流类型
 */
ob_stream_type ob_imu_frame_get_stream_type(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧数据
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return void* 返回帧数据指针
 */
void *ob_frame_data(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧数据大小
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧大小
 */
uint32_t ob_frame_data_size(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧的流类型
 * 
 * @param[in] frame 帧对象
 * @param[out] error 记录错误信息
 * @return ob_stream_type 返回流类型
 */
ob_stream_type ob_frame_get_stream_type(ob_frame *frame, ob_error **error);

/**
 * @brief 对帧使用过滤器
 * 
 * @param[in] frame 帧对象
 * @param[in] filter 要使用的过滤器
 * @param[out] error 记录错误信息
 */
//void ob_frame_apply_filter(ob_frame *frame, ob_filter *filter, ob_error **error);

/**
 * @brief 删除帧
 * 
 * @param[in] frame 要删除的帧对象
 * @param[out] error 记录错误信息
 */
void ob_delete_frame(ob_frame *frame, ob_error **error);

/**
 * @brief 获取帧集合包含的帧数量
 * 
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回帧数量
 */
uint32_t ob_frame_set_frame_count(ob_frame_set *frame_set, ob_error **error);

/**
 * @brief 从帧集合中获取深度帧
 * 
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回深度帧
 */
ob_frame *ob_frame_set_depth_frame(ob_frame_set *frame_set, ob_error **error);

/**
 * @brief 从帧集合中获取彩色帧
 * 
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回彩色帧
 */
ob_frame *ob_frame_set_color_frame(ob_frame_set *frame_set, ob_error **error);

/**
 * @brief 从帧集合中获取红外帧
 * 
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回红外帧
 */
ob_frame *ob_frame_set_infrared_frame(ob_frame_set *frame_set, ob_error **error);
/**
 * @brief 从帧集合中获取辅助帧
 *
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回辅助帧
 */
ob_frame *ob_frame_set_assist_frame(ob_frame_set *frame_set, ob_error **error);
/**
 * @brief 通过传感器类型获取帧
 * 
 * @param[in] frame_set 帧集合对象
 * @param[in] type 传感器的类型
 * @param[out] error 记录错误信息
 * @return ob_frame* 返回相应类型的帧
 */
//ob_frame *ob_frame_set_get_frame(ob_frame_set *frame_set, ob_sensor_type type, ob_error **error);
/**
 * @brief 删除帧集合
 * 
 * @param[in] frame_set 帧集合对象
 * @param[out] error 记录错误信息
 */
void ob_delete_frame_set(ob_frame_set *frame_set, ob_error **error);

#ifdef __cplusplus
}
#endif
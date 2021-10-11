/**
 * @file Pipeline.h
 * @brief SDK的高级API，可以快速实现开关流，帧同步，软件滤波等功能，适用于应用，算法重点关于rgbd数据流场景。如果对实时性或需要单独
 * 处理的同步，对齐的场景。请使用Device的Lower API的接口。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 创建pipeline对象
 *
 * @param[out] error 记录错误信息
 * @return ob_pipeline* 返回pipeline对象
 */
ob_pipeline* ob_create_pipeline( ob_error** error );

/**
 * @brief 使用设备对象来创建pipeline对象
 *
 * @param[in] dev 用于创建pipeline的设备对象
 * @param[out] error 记录错误信息
 * @return ob_pipeline* 返回pipeline对象
 */
ob_pipeline* ob_create_pipeline_with_device( ob_device* dev, ob_error** error );

/**
 * @brief 删除pipeline对象
 *
 * @param[in] pipeline 要删除的pipeline对象
 * @param[out] error 记录错误信息
 */
void ob_delete_pipeline( ob_pipeline* pipeline, ob_error** error );

/**
 * @brief 以默认参数启动pipeline
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 */
void ob_pipeline_start( ob_pipeline* pipeline, ob_error** error );

/**
 * @brief 启动pipeline并配置参数
 *
 * @param[in] pipeline pipeline对象
 * @param[in] config 要配置的参数
 * @param[out] error 记录错误信息
 */
void ob_pipeline_start_with_config( ob_pipeline* pipeline, ob_config* config, ob_error** error );

/**
 * @brief 启动pipeline并设置帧集合数据回调
 *
 * @param[in] pipeline pipeline对象
 * @param[in] config 要配置的参数
 * @param[in] callback 帧集合中的所有帧数据都到达时触发回调
 * @param[in] user_data 可以传入任意用户数据，并从回调中获取
 * @param[out] error 记录错误信息
 */
void ob_pipeline_start_with_callback( ob_pipeline* pipeline, ob_config* config, ob_frame_set_callback callback, void* user_data, ob_error** error );

/**
 * @brief 停止pipeline
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 */
void ob_pipeline_stop( ob_pipeline* pipeline, ob_error** error );

/**
 * @brief 获取pipeline当前参数
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * @return ob_config* 返回pipeline参数
 */
ob_config* ob_pipeline_get_config( ob_pipeline* pipeline, ob_error** error );

/**
 * @brief 以同步阻塞的形式等待返回一组帧集合
 *
 * @param[in] pipeline pipeline对象
 * @param[in] timeout_ms 等待超时时间(毫秒)
 * @param[out] error 记录错误信息
 * @return ob_frame_set* 返回等待的帧集合
 */
ob_frame_set* ob_pipeline_wait_for_frames( ob_pipeline* pipeline, uint32_t timeout_ms, ob_error** error );

/**
 * @brief 从pipeline中获取设备
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 * @return ob_device* 返回设备对象
 */
ob_device* ob_pipeline_get_device( ob_pipeline* pipeline, ob_error** error );

/**
 * @brief 获取传感器相应的流配置
 *
 * @param[in] pipeline pipeline对象
 * @param[in] sensor_type 传感器类型
 * @param[out] profile_count 获取的流配置数量
 * @param[out] error 记录错误信息
 * @return ob_stream_profile** 返回流配置列表
 */
ob_stream_profile** ob_pipeline_get_stream_profiles( ob_pipeline* pipeline, ob_sensor_type sensor_type, uint32_t* profile_count, ob_error** error );

/**
 * @brief 获取所有流配置
 *
 * @param[in] pipeline pipeline对象
 * @param[out] profile_count 获取的流配置数量
 * @param[out] error 记录错误信息
 * @return ob_stream_profile** 返回流配置列表
 */
ob_stream_profile** ob_pipeline_get_all_stream_profiles( ob_pipeline* pipeline, uint32_t* profile_count, ob_error** error );

/**
 * @brief 打开帧同步功能
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 */
void ob_pipeline_enable_frame_sync( ob_pipeline* pipeline, ob_error** error );

/**
 * @brief 关闭帧同步功能
 *
 * @param[in] pipeline pipeline对象
 * @param[out] error 记录错误信息
 */
void ob_pipeline_disable_frame_sync( ob_pipeline* pipeline, ob_error** error );

/**
 * @brief 创建pipeline的配置
 *
 * @param[out] error 记录错误信息
 * @return ob_config* 返回配置对象
 */
ob_config* ob_create_config( ob_error** error );

/**
 * @brief 删除pipeline的配置
 *
 * @param[in] config 要删除的配置
 * @param[out] error 记录错误信息
 */
void ob_delete_config( ob_config* config, ob_error** error );

/**
 * @brief 配置要打开的流
 *
 * @param[in] config pipeline的配置
 * @param[in] profile 要打开的流的配置
 * @param[out] error 记录错误信息
 */
void ob_config_enable_stream( ob_config* config, ob_stream_profile* profile, ob_error** error );

/**
 * @brief 配置打开所有的流
 *
 * @param[in] config pipeline的配置
 * @param[out] error 记录错误信息
 */
void ob_config_enable_all_stream( ob_config* config, ob_error** error );

/**
 * @brief 配置要关闭的流
 *
 * @param[in] config pipeline的配置
 * @param[in] profile 要关闭的流的配置
 * @param[out] error 记录错误信息
 */
void ob_config_disable_stream( ob_config* config, ob_stream_type type, ob_error** error );

/**
 * @brief 配置关闭所有的流
 *
 * @param[in] config pipeline的配置
 * @param[out] error 记录错误信息
 */
void ob_config_disable_all_stream( ob_config* config, ob_error** error );

/**
 * @brief  Filter重置
 *
 * @param[in] filter filter对象
 * @param[out] error 记录错误信息
 */
void ob_filter_reset( ob_filter* filter, ob_error** error );

/**
 * @brief Filter 处理(同步接口)
 *
 * @param[in] filter filter对象
 * @param[in/out] frame_set 需要被处理的frameset对象指针，处理结果会覆盖该对象通过该对象指针返回
 * @param[out] error 记录错误信息
 */
void ob_filter_process( ob_filter* filter, ob_frame* frame, ob_error** error );

/**
 * @brief Filter 启动处理线程(异步回调接口)
 *
 * @param[in] filter filter对象
 * @param[out] error 记录错误信息
 */
bool ob_filter_start( ob_filter* filter, ob_error** error );

/**
 * @brief Filter 停止处理线程(异步回调接口)
 *
 * @param[in] filter filter对象
 * @param[out] error 记录错误信息
 */
void ob_filter_stop( ob_filter* filter, ob_error** error );

/**
 * @brief Filter 设置处理结果回调函数(异步回调接口)
 *
 * @param[in] filter filter对象
 * @param[in] callback 回调函数
 * @param[in] user_data 可以传入任意用户数据指针，并从回调返回该数据指针
 * @param[out] error 记录错误信息
 */
void ob_filter_set_callback( ob_filter* filter, ob_filter_callback callback, void* user_data, ob_error** error );

/**
 * @brief filter 压入frame_set到待处理缓存(异步回调接口)
 *
 * @param[in] filter filter对象
 * @param[out] error 记录错误信息
 */
void ob_filter_push_frame( ob_filter* filter, ob_frame* frame, ob_error** error );

#ifdef __cplusplus
}
#endif
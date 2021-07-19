/**
 * @file Context.h
 * @brief 底层SDK的入口，包括创建SDK上下文环境，获取设备列表，处理设备的回调，日志等级的设置操作
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 创建上下文环境
 * 
 * @param[out] error 记录错误信息
 * @return ob_context* 返回上下文环境
 */
ob_context* ob_create_context( ob_error** error );

/**
 * @brief 删除上下文环境
 * 
 * @param[in] context 要删除的上下文环境
 * @param[out] error 记录错误信息
 */
void ob_delete_context( ob_context* context, ob_error** error );

/**
 * @brief 获取设备列表
 * 
 * @param[in] context 上下文环境
 * @param[out] error 记录错误信息
 * @return ob_device_list* 返回设备列表对象
 */
ob_device_list* ob_query_device_list( ob_context* context, ob_error** error );

/**
 * @brief 设置设备插拔回调函数
 * 
 * @param[in] context 上下文环境
 * @param[in] callback 设备插拔时触发的回调
 * @param[in] user_data 可以传入任意用户数据，并从回调中获取
 * @param[out] error 记录错误信息
 */
void ob_set_device_changed_callback( ob_context* context, ob_device_changed_callback callback,
                   void *user_data, ob_error **error);

/**
 * @brief 设置日志的等级
 * 
 * @param[in] context 上下文环境
 * @param[in] log 日志的等级
 * @param[out] error 记录错误信息
 */
void ob_set_logger_serverity( ob_context* context, ob_log_severity log, ob_error** error );

/**
 * @brief 设置输出日志到文件
 * 
 * @param[in] context 上下文环境
 * @param[in] log 日志的等级
 * @param[in] file_name 日志文件名
 * @param[out] error 记录错误信息
 */
void ob_set_logger_to_file( ob_context* context, ob_log_severity log, const char* file_name, ob_error** error );

/**
 * @brief 设置输出日志到控制台
 * 
 * @param[in] context 上下文环境
 * @param[in] log 日志的等级
 * @param[out] error 记录错误信息
 */
void  ob_set_logger_to_console( ob_context* context, ob_log_severity log, ob_error **error);

#ifdef __cplusplus
}
#endif
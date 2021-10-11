/**
 * @file Context.h
 * @brief context是描述SDK的runtime一个管理类，负责SDK的资源申请与释放
 * context具备多设备的管理能力，负责枚举设备，监听设备回调，启用多设备同步等功能
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 创建context的接口函数
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
 * @brief 获取OS枚举设备列表
 *
 * @param[in] context 上下文环境
 * @param[out] error 记录错误信息
 * @return ob_device_list* 返回设备列表对象
 */
ob_device_list* ob_query_device_list( ob_context* context, ob_error** error );

/**
 * @brief 设置设备插拔回调函数
 * @attention 通过回调接口返回的added和removed设备列表，在回调函数返回后会自动销毁
 *
 * @param[in] context 上下文环境
 * @param[in] callback 设备插拔时触发的回调
 * @param[in] user_data 可以传入任意用户数据，并从回调中获取
 * @param[out] error 记录错误信息
 */
void ob_set_device_changed_callback( ob_context* context, ob_device_changed_callback callback, void* user_data, ob_error** error );

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
void ob_set_logger_to_console( ob_context* context, ob_log_severity log, ob_error** error );

/**
 * @brief 启动多设备同步功能，同步已创建设备的时钟(需要使用的设备支持该功能)
 *
 * @param[in]  context 上下文环境
 * @param[in]  repeatInterval 定时同步时间间隔（单位ms；如果repeatInterval=0，表示只同步一次，不再定时执行）
 * @param[out] error 记录错误信息
 */
void ob_enable_multi_device_sync( ob_context* context, uint64_t repeatInterval, ob_error** error );

#ifdef __cplusplus
}
#endif
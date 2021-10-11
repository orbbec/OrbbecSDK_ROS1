/**
 * @file StreamProfile.h
 * @brief 流配置相关函数，用于获取流的宽、高、帧率及格式等信息
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 获取流配置的帧率
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的帧率
 */
uint32_t ob_stream_profile_fps( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取流配置的宽
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的宽
 */
uint32_t ob_stream_profile_width( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取流配置的高
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return uint32_t 返回流的高
 */
uint32_t ob_stream_profile_height( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取流配置的格式
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return ob_format 返回流的格式
 */
ob_format ob_stream_profile_format( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 获取流的类型
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 * @return ob_stream_type 流的类型
 */
ob_stream_type ob_stream_profile_type( ob_stream_profile* profile, ob_error** error );

/**
 * @brief 删除流配置列表
 *
 * @param[in] profiles 流配置列表
 * @param[in] count 流配置数量
 * @param[out] error 记录错误信息
 */
void ob_delete_stream_profiles( ob_stream_profile** profiles, uint32_t count, ob_error** error );

/**
 * @brief 删除流配置
 *
 * @param[in] profile 流配置对象
 * @param[out] error 记录错误信息
 */
void ob_delete_stream_profile( ob_stream_profile* profile, ob_error** error );

#ifdef __cplusplus
}
#endif
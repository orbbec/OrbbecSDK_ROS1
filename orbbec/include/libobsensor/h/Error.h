/**
 * @file Error.h
 * @brief 错误处理相关函数，主要用于获取错误信息
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief 获取错误状态
 * 
 * @param[in] error 错误对象
 * @return ob_status 返回错误状态
 */
ob_status ob_error_status(ob_error* error);
/**
 * @brief 获取错误信息
 * 
 * @param[in] error 错误对象
 * @return const char* 返回错误信息
 */
const char* ob_error_message(ob_error* error);
/**
 * @brief 获取错误函数
 * 
 * @param[in] error 错误对象
 * @return const char* 返回错误函数
 */
const char* ob_error_function(ob_error* error);
/**
 * @brief 获取错误参数
 * 
 * @param[in] error 错误对象
 * @return const char* 返回错误参数
 */
const char* ob_error_args(ob_error* error);
/**
 * @brief 获取错误异常类型
 * 
 * @param[in] error 错误对象
 * @return ob_exception_type 返回错误异常类型
 */
ob_exception_type ob_error_exception_type(ob_error* error);
/**
 * @brief 删除错误对象
 * 
 * @param[in] error 要删除的错误对象
 */
void ob_delete_error(ob_error* error);

#ifdef __cplusplus
}
#endif
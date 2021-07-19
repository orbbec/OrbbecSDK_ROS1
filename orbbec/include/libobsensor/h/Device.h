/**
 * @file Device.h
 * @brief 设备相关函数，包括获取创建设备，设置及获取设备属性，获取传感器等操作
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
* @brief 获取设备数量
*
* @param[in] list 设备列表对象
* @param[out] error 记录错误信息
* @return uint32_t 返回设备数量
*/
uint32_t ob_device_list_device_count(ob_device_list* list, ob_error** error);

/**
 * @brief 获取设备信息
 * 
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return ob_device_info* 返回设备信息
 */
ob_device_info *
ob_device_list_get_device_info(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief 获取指定设备名称
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return const char* 返回设备名称
 */
const char* ob_device_list_get_device_name(ob_device_list* list, uint32_t index, ob_error** error);

/**
 * @brief 获取指定设备pid
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return int 返回设备pid
 */
int ob_device_list_get_device_pid(ob_device_list* list, uint32_t index, ob_error** error);

/**
 * @brief 获取指定设备vid
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return int 返回设备vid
 */
int ob_device_list_get_device_vid(ob_device_list* list, uint32_t index, ob_error** error);

/**
 * @brief 获取指定设备uid
 *
 * @param[in] list 设备列表对象
 * @param[in] index 设备索引
 * @param[out] error 记录错误信息
 * @return const char* 返回设备uid
 */
const char* ob_device_list_get_device_uid(ob_device_list* list, uint32_t index, ob_error** error);

/**
 * @brief 创建设备
 * 
 * @param[in] list 设备列表对象
 * @param[in] info 要创建设备的设备信息
 * @param[out] error 记录错误信息
 * @return ob_device* 返回创建的设备
 */
// ob_device *ob_device_list_create_device(ob_device_list *list, ob_device_info *info,
//                                              ob_error **error);

/**
 * @brief 创建设备
 * 
 * @param[in] list 设备列表对象
 * @param index 要创建设备的索引
 * @param[out] error 记录错误信息
 * @return ob_device* 返回创建的设备
 */
ob_device *ob_device_list_create_device(ob_device_list *list, uint32_t index, ob_error **error);

/**
 * @brief 删除设备
 * 
 * @param[in] device 要删除的设备
 * @param[out] error 记录错误信息
 */
void ob_delete_device(ob_device *device, ob_error **error);

/**
 * @brief 删除设备信息
 * 
 * @param[int] info 要删除的设备信息
 * @param[out] error 记录错误信息
 */
void ob_delete_device_info(ob_device_info *info, ob_error **error);

/**
 * @brief 删除设备列表
 * 
 * @param[in] list 要删除的设备列表对象
 * @param[out] error 记录错误信息
 */
void ob_delete_device_list(ob_device_list *list, ob_error **error);

/**
 * @brief 获取设备信息
 * 
 * @param[in] device 要获取信息的设备
 * @param[out] error 记录错误信息
 * @return ob_device_info* 返回设备信息
 */
ob_device_info *ob_device_get_device_info(ob_device *device, ob_error **error);

/**
 * @brief 获取设备的所有传感器
 * 
 * @param[in] device 设备对象
 * @param[out] count 获取的传感器数量
 * @param[out] error 记录错误信息
 * @return ob_sensor** 返回所有传感器
 */
ob_sensor** ob_device_get_sensors(ob_device* device, uint32_t* count, ob_error** error);

/**
 * @brief 获取设备的传感器
 * 
 * @param[in] device 设备对象
 * @param[in] type 要获取的传感器类型
 * @param[out] error 记录错误信息
 * @return ob_sensor* 返回获取的传感器
 */
ob_sensor *ob_device_get_sensor(ob_device *device, ob_sensor_type type, ob_error **error);

/**
 * @brief 设置int类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 */
void ob_device_set_int_property(ob_device* device, ob_global_unified_property property_id, int32_t property, 
    ob_error** error);

/**
 * @brief 获取int类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return int32_t 返回属性值
 */
int32_t ob_device_get_int_property(ob_device* device, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief 设置float类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 */
void ob_device_set_float_property(ob_device* device, ob_global_unified_property property_id, float property,
    ob_error** error);

/**
 * @brief 获取float类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return int32_t 返回属性值
 */
float ob_device_get_float_property(ob_device* device, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief 设置bool类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] property 要设置的属性值
 * @param[out] error 记录错误信息
 */
void ob_device_set_bool_property(ob_device* device, ob_global_unified_property property_id, bool property, 
    ob_error** error);

/**
 * @brief 获取bool类型的设备属性
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return bool 返回属性值
 */
bool ob_device_get_bool_property(ob_device* device, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief 设置结构体类型的设备属性
 * 
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] data 要设置的属性数据
 * @param[in] data_size 要设置的属性大小
 * @param[out] error 记录错误信息
 */
void ob_device_set_structured_data(ob_device *device, ob_global_unified_property property_id, const void *data,
    uint32_t data_size, ob_error **error);

/**
 * @brief 获取结构体类型的设备属性
 * 
 * @param[in] device 设备对象
 * @param[in] property_id 要获取的属性id
 * @param[out] data 获取的属性数据
 * @param[out] data_size 获取的属性大小
 * @param[out] error 记录错误信息
 */
void ob_device_get_structured_data( ob_device* device, ob_global_unified_property property_id, void* data,
    uint32_t *data_size, ob_error **error);
    
/**
 * @brief 设置raw data类型d 设备属性
 * 
 * @param[in] device 设备对象
 * @param[in] property_id 要设置的属性id
 * @param[in] data 要设置的属性数据
 * @param[in] data_size 要设置的属性大小
 * @param[in] cb 设置进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回 
 * @param[out] error 记录错误信息
 */
bool ob_device_set_raw_data(
    ob_device *device, ob_global_unified_property property_id, 
    void *data, uint32_t data_size, 
    ob_set_data_callback cb, bool async,
    void *user_data, ob_error **error
    );

/**
 * @brief 获取raw data类型的设备属性
 * 
 * @param[in] device 设备对象
 * @param[in] property_id 要获取的属性id
 * @param[in] cb 获取数据给返回和进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回 
 * @param[out] error 记录错误信息
 */
bool ob_device_get_raw_data(
    ob_device *device, ob_global_unified_property property_id, 
    ob_get_data_callback cb, bool async,
    void *user_data, ob_error **error
    );

/**
 * @brief 判断设备属性是否支持
 * 
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return bool 返回是否支持
 */
bool
ob_device_is_property_supported(ob_device *device, ob_global_unified_property property_id, ob_error **error);

/**
 * @brief 获取int类型的设备属性范围
 * 
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return ob_int_property_range 返回属性范围
 */
ob_int_property_range
ob_device_get_int_property_range(ob_device *device, ob_global_unified_property property_id, ob_error **error);

/**
 * @brief 获取float类型的设备属性范围
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return ob_float_property_range 返回属性范围
 */
ob_float_property_range
ob_device_get_float_property_range(ob_device* device, ob_global_unified_property property_id, ob_error** error);


/**
 * @brief 获取bool类型的设备属性范围
 *
 * @param[in] device 设备对象
 * @param[in] property_id 属性id
 * @param[out] error 记录错误信息
 * @return ob_bool_property_range 返回属性范围
 */
ob_bool_property_range
ob_device_get_bool_property_range(ob_device* device, ob_global_unified_property property_id, ob_error** error);

/**
 * @brief ahb写寄存器
 *
 * @param[in] device 设备对象
 * @param reg 要写入的寄存器
 * @param mask 掩码
 * @param value 要写入的值
 * @param[out] error 记录错误信息
 */
void ob_device_write_ahb( ob_device* device, uint32_t reg, uint32_t mask, uint32_t value, ob_error** error );

/**
 * @brief ahb读寄存器
 *
 * @param[in] device 设备对象
 * @param reg 要读取的寄存器
 * @param mask 掩码
 * @param value 要读取的值
 * @param[out] error 记录错误信息
 */
void ob_device_read_ahb( ob_device* device, uint32_t reg, uint32_t mask, uint32_t* value, ob_error** error );


/**
 * @brief i2c写寄存器
 *
 * @param[in] device 设备对象
 * @param module_id 要写入的i2c模块id
 * @param reg 要写入的寄存器
 * @param mask 掩码
 * @param value 要写入的值
 * @param[out] error 记录错误信息
 */
void ob_device_write_i2c( ob_device* device, uint32_t module_id, uint32_t reg, uint32_t mask, uint32_t value, ob_error** error );

/**
 * @brief i2c读寄存器
 *
 * @param[in] device 设备对象
 * @param module_id 要读取的的i2c模块id
 * @param reg 要读取的寄存器
 * @param mask 掩码
 * @param value 要读取的值
 * @param[out] error 记录错误信息
 */
void ob_device_read_i2c( ob_device* device,  uint32_t module_id, uint32_t reg, uint32_t mask, uint32_t* value, ob_error** error );

/**
 * @brief 设置写入Flash的属性[异步回调]
 *
 * @param[in] device 设备对象
 * @param offset flash偏移地址
 * @param data 要写入的属性数据
 * @param data_size 要写入的属性大小
 * @param cb 写进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回 
 * @param[out] error 记录错误信息
 */
bool ob_device_write_flash(
    ob_device* device, uint32_t offset, 
    const void* data, uint32_t data_size,
    ob_set_data_callback cb, bool async,
    void *user_data, ob_error** error
);

/**
 * @brief 读取Flash的属性[异步回调]
 *
 * @param[in] device 设备对象
 * @param offset flash偏移地址
 * @param data_size 要读取的数据大小
 * @param cb 读flash数据及进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回 
 * @param[out] error 记录错误信息
 */
bool ob_device_read_flash(
    ob_device* device, uint32_t offset, 
    uint32_t data_size, 
    ob_get_data_callback cb, bool async,
    void *user_data, ob_error** error 
);

/**
 * @brief 更新设备时间戳（向设备授时，同步本地系统时间到设备）
 *
 * @param[in] device 设备对象
 * @param[out] uint64_t 命令往返时间延时（round trip time， rtt）
 * @param[out] error 记录错误信息
 */
uint64_t ob_device_update_device_time(ob_device* device,  ob_error** error );

/**
 * @brief 设备固件升级
 * 
 * @param[in] device 设备对象
 * @param[in] path 固件路径
 * @param[in] callback 固件升级进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回 
 * @param[out] error 记录错误信息
 * @return bool 接口调用是否成功 -> 0: 失败，底层接口占用或线程占用；1: 成功
 */
bool ob_device_upgrade( 
    ob_device *device, 
    const char *path, 
    ob_device_upgrade_callback callback, 
    bool async,
    void *user_data, 
    ob_error **error 
    );

/**
 * @brief 设置设备状态监听
 * 
 * @param[in] device 设备对象
 * @param[in] callback 设备状态发生改变时的回调
 * @param[in] user_data 用户自定义数据，会在回调中返回
 * @param[out] error 记录错误信息
 */
void ob_device_state_changed(ob_device* device, ob_device_state_callback callback, void* user_data, ob_error** error);

/**
 * @brief 重置设备的所有属性
 * 
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 */
void ob_device_reset(ob_device *device, ob_error **error);

/**
 * @brief 发送文件到设备指定路径
 * 
 * @param[in] device 设备对象
 * @param[in] file_path 源文件路径
 * @param[in] dst_path 指定设备端文件接受路径
 * @param[in] callback 文件发送进度回调
 * @param[in] async 是否异步执行
 * @param[in] user_data 用户自定义数据，会在回调中返回 
 * @param[out] error 记录错误信息
 * @return bool 接口调用是否成功 -> 0: 失败，底层接口占用或线程占用；1: 成功
 */
bool ob_device_send_file_to_destination(ob_device *device, const char *file_path, const char *dst_path, ob_file_send_callback callback, bool async, void *user_data, ob_error **error );

/**
 * @brief 验证设备授权码
 * 
 * @param[in] device 设备对象
 * @param[in] auth_code 授权码
 * @param[out] error 记录错误信息
 * @return bool 激活是否成功
 */
bool ob_device_activate_authorization(ob_device *device, const char* auth_code, ob_error **error);

/**
 * @brief 写入设备授权码
 *
 * @param[in] device 设备对象
 * @param[in] auth_code 授权码
 * @param[out] error 记录错误信息
 */
void ob_device_write_authorization_code(ob_device* device, const char* auth_code, ob_error** error);

/**
 * @brief 获取相机内参（会根据当前镜像状态返回）
 *
 * @param[in] device 设备对象
 * @param[in] sensor_type sensor类型
 * @param[out] error 记录错误信息
 * 
 * @return ob_camera_distortion 畸变参数结构体
 */
ob_camera_intrinsic ob_device_get_camera_intrinsic(ob_device* device, ob_sensor_type sensor_type, ob_error** error);

/**
 * @brief 获取相机畸变参数（会根据当前镜像状态返回）
 *
 * @param[in] device 设备对象
 * @param[in] sensor_type sensor类型
 * @param[out] error 记录错误信息
 *  
 * @return ob_camera_distortion 畸变参数结构体
 */
ob_camera_distortion ob_device_get_camera_distortion(ob_device* device, ob_sensor_type sensor_type, ob_error** error);

/**
 * @brief 获取d2c旋转矩阵（会根据当前镜像状态返回）
 *
 * @param[in] device 设备对象
 * @param[out] error 记录错误信息
 * 
 * @return ob_d2c_transform 旋转矩阵结构体
 */
ob_d2c_transform ob_device_get_d2c_transform(ob_device* device, ob_sensor_type sensor_type, ob_error** error);


/**
 * @brief 获取设备名
 * 
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回设备名
 */
const char *ob_device_info_name(ob_device_info *info, ob_error **error);

/**
 * @brief 获取设备pid
 * 
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return int 返回设备pid
 */
int ob_device_info_pid(ob_device_info *info, ob_error **error);

/**
 * @brief 获取设备vid
 * 
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return int 返回设备vid
 */
int ob_device_info_vid(ob_device_info *info, ob_error **error);

/**
 * @brief 获取设备uid
 * 
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回设备uid
 */
const char *ob_device_info_uid(ob_device_info *info, ob_error **error);

/**
 * @brief 获取设备序列号
 * 
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return const char* 返回设备序列号
 */
const char *ob_device_info_serial_number(ob_device_info *info, ob_error **error);

/**
 * @brief 获取固件版本号
 * 
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return int 返回固件版本号
 */
const char* ob_device_info_firmware_version(ob_device_info *info, ob_error **error);

/**
 * @brief 获取usb连接类型
 * 
 * @param[in] info 设备信息
 * @param[out] error 记录错误信息
 * @return char* 返回usb连接类型
 */
const char* ob_device_info_usb_type(ob_device_info *info, ob_error **error);


#ifdef __cplusplus
}
#endif
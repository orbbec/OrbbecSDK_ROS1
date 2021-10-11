#include <exception>
#include "Types.hpp"

#pragma once
namespace ob {
/**
 * @brief Error描述SDK内部的异常错误，通过该类可以实现异常的详细信息。
 */
class Error : public std::exception {
public:
    /**
     * @brief 获取SDK内部异常的详细错误日志。
     */
    const char* getMessage() const noexcept {
        return msg_.c_str();
    }
    /**
     * @brief 获取该错误的异常类型，判断是具体哪个模块异常。
     * @link OBExceptionType
     */
    OBExceptionType getExceptionType() const noexcept {
        return exceptionType_;
    }
    /**
     * @brief 获取SDK内部异常的错误接口函数名称。
     */
    const char* getName() const noexcept {
        return name_;
    }
    /**
     * @brief 获取SDK内部异常的错误接口函数传入参数。
     */
    const char* getArgs() const noexcept {
        return args_.c_str();
    }

public:
    Error( const char* name, const std::string& args, const std::string& msg, OBExceptionType exceptionType ) noexcept : name_( name ), args_( args ), msg_( msg ), exceptionType_( exceptionType ) {}

private:
    const char*     name_;
    std::string     args_;
    std::string     msg_;
    OBExceptionType exceptionType_;
};
}  // namespace ob

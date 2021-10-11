/**
 * @file Version.hpp
 * @brief 获取SDK版本号相关信息的类
 *
 */

namespace ob {
class OB_EXTENSION_API Version {
public:
    /**
     * @brief 获取SDK主版本号
     *
     * @return int 返回SDK主版本号
     */
    static int major();
    /**
     * @brief 获取SDK副版本号
     *
     * @return int 返回SDK副版本号
     */
    static int minor();
    /**
     * @brief 获取SDK修订版本号
     *
     * @return int 返回SDK修订版本号
     */
    static int patch();

    /**
     * @brief 获取SDK版本号
     *
     * @return int 返回SDK版本号
     */
    static int version();
};
}  // namespace ob
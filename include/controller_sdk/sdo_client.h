#pragma once

/**
 * @file sdo_client.h
 * @brief NRT SDO 客户端（对象字典读写）
 *
 * ## 契约
 * - **仅 NRT**：内部 `usleep` 轮询，禁止在实时周期内调用。
 * - 按 `ec_slave_config_t*` 缓存 `ec_sdo_request_t`，线程安全（单把操作锁）。
 * - 与 PDO 周期路径分离；电机配置（轮廓速度/回零方式等）应走本类。
 *
 * CiA402Motor 内部委托本类；亦可对任意从站独立构造使用。
 */

#include <ecrt.h>

#include <cstdint>
#include <cstddef>
#include <map>
#include <mutex>

class SdoClient {
public:
    /** @brief SDO 操作结果 */
    enum class Result : uint8_t {
        Ok = 0,           ///< 成功
        NullSlave,        ///< 从站配置为空
        CreateFailed,     ///< 创建请求失败
        RequestError,     ///< 请求错误 / abort
        Timeout           ///< 超时
    };

    SdoClient() = default;
    /**
     * @brief 绑定从站配置
     * @param slave_config IgH 从站配置句柄
     */
    explicit SdoClient(ec_slave_config_t* slave_config) : sc_(slave_config) {}

    /**
     * @brief 更换绑定的从站
     * @param slave_config 句柄；可为 nullptr
     */
    void setSlaveConfig(ec_slave_config_t* slave_config) noexcept { sc_ = slave_config; }
    /** @brief 当前从站配置句柄 */
    ec_slave_config_t* slaveConfig() const noexcept { return sc_; }

    /**
     * @brief SDO 下载（写对象字典）
     * @param index 索引
     * @param subindex 子索引
     * @param data 数据指针
     * @param size 字节数
     * @param timeout_ms 超时
     * @return 结果码
     */
    Result download(uint16_t index, uint8_t subindex,
                    const void* data, size_t size,
                    uint32_t timeout_ms = 500);

    /**
     * @brief SDO 上传（读对象字典）
     * @param index 索引
     * @param subindex 子索引
     * @param data 接收缓冲
     * @param size 缓冲字节数
     * @param timeout_ms 超时
     * @return 结果码
     */
    Result upload(uint16_t index, uint8_t subindex,
                  void* data, size_t size,
                  uint32_t timeout_ms = 500);

    /**
     * @brief 写对象字典（bool 版）
     * @return true 表示 `Result::Ok`
     */
    bool write(uint16_t index, uint8_t subindex,
               const void* data, size_t size,
               uint32_t timeout_ms = 500) {
        return download(index, subindex, data, size, timeout_ms) == Result::Ok;
    }

    /**
     * @brief 读对象字典（bool 版）
     * @return true 表示 `Result::Ok`
     */
    bool read(uint16_t index, uint8_t subindex,
              void* data, size_t size,
              uint32_t timeout_ms = 500) {
        return upload(index, subindex, data, size, timeout_ms) == Result::Ok;
    }

    /**
     * @brief 按类型写对象
     * @tparam T 值类型（按 `sizeof(T)` 发送）
     */
    template <typename T>
    bool writeValue(uint16_t index, uint8_t subindex, const T& value,
                    uint32_t timeout_ms = 500) {
        return write(index, subindex, &value, sizeof(T), timeout_ms);
    }

    /**
     * @brief 按类型读对象
     * @tparam T 值类型
     * @param[out] value 读出值
     */
    template <typename T>
    bool readValue(uint16_t index, uint8_t subindex, T& value,
                   uint32_t timeout_ms = 500) {
        return read(index, subindex, &value, sizeof(T), timeout_ms);
    }

    /**
     * @brief 结果码转短字符串
     * @param r 结果
     * @return 静态 C 字符串
     */
    static const char* resultToCStr(Result r) noexcept;

private:
    ec_sdo_request_t* getOrCreateRequest(uint16_t index, uint8_t subindex, size_t size);

    ec_slave_config_t* sc_ = nullptr;
    std::mutex op_mutex_;
    std::map<uint32_t, ec_sdo_request_t*> cache_;
};

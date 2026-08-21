#pragma once

/**
 * @file i_digital_io.h
 * @brief 数字 IO 抽象
 *
 * ## RT / NRT 契约
 * - **RT**：读写过程数据映射的输入/输出字与位。
 * - **NRT**：配置偏移、绑定 Domain 指针（见实现类 `DigitalIo`）。
 *
 * 默认实现：`DigitalIo`（常见对象 0x60FD / 0x60FE）。
 */

#include <cstdint>

/**
 * @class IDigitalIo
 * @brief 数字量输入/输出接口（过程数据字 + 位操作）
 */
class IDigitalIo {
public:
    virtual ~IDigitalIo() = default;

    /**
     * @brief 读数字输入字
     * @return 输入位图；未映射时返回 0
     */
    virtual uint32_t readInputs() const noexcept = 0;

    /**
     * @brief 读当前数字输出字（回读过程数据）
     * @return 输出位图；未映射时返回 0
     */
    virtual uint32_t readOutputs() const noexcept = 0;

    /**
     * @brief 写数字输出字
     * @param value 输出位图；未映射时为 no-op
     */
    virtual void writeOutputs(uint32_t value) noexcept = 0;

    /**
     * @brief 读单个输入位
     * @param bit 位号（0 为最低位）
     * @return true 该位为 1
     */
    virtual bool getInputBit(unsigned bit) const noexcept = 0;

    /**
     * @brief 读单个输出位
     * @param bit 位号（0 为最低位）
     * @return true 该位为 1
     */
    virtual bool getOutputBit(unsigned bit) const noexcept = 0;

    /**
     * @brief 置位/清零单个输出位（其余位保持）
     * @param bit 位号
     * @param on true 置 1，false 清 0
     */
    virtual void setOutputBit(unsigned bit, bool on) noexcept = 0;
};

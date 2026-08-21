#pragma once

/**
 * @file digital_io.h
 * @brief 基于 Domain 过程数据的数字 IO（如 0x60FD / 0x60FE）
 *
 * ## RT / NRT 契约
 * - **RT**：`readInputs` / `writeOutputs` / 位操作。
 * - **NRT**：`setDomainData` / `setOffsets`（activate 后绑定）。
 *
 * Offsets 由调用方在 PDO 注册后填入（与 `CiA402Motor::Offsets` 同模式：
 * `UNMAPPED = UINT_MAX`）。未映射的通道读写为 no-op / 返回 0。
 */

#include "i_digital_io.h"
#include "CiA402Motor.h"

#include <climits>
#include <cstdint>

/**
 * @class DigitalIo
 * @brief `IDigitalIo` 默认实现：直接读写 Domain PD
 */
class DigitalIo : public IDigitalIo {
public:
    /** @brief PDO 字节偏移；未映射为 `UNMAPPED` */
    struct Offsets {
        static constexpr unsigned int UNMAPPED = UINT_MAX;
        unsigned int digital_inputs = UNMAPPED;   ///< 常见 0x60FD:00
        unsigned int digital_outputs = UNMAPPED;  ///< 常见 0x60FE:01

        /**
         * @brief 偏移是否已映射
         * @param offset PDO 字节偏移
         * @return true 已映射
         */
        static constexpr bool isMapped(unsigned int offset) noexcept {
            return offset != UNMAPPED;
        }

        /**
         * @brief 从电机 PDO Offsets 提取 IO 字段
         * @param m 电机偏移（XML 映射后）
         * @return IO 偏移
         */
        static Offsets fromMotorOffsets(const CiA402Motor::Offsets& m) noexcept {
            Offsets o;
            o.digital_inputs = m.digital_inputs;
            o.digital_outputs = m.digital_outputs;
            return o;
        }
    };

    DigitalIo() = default;
    /**
     * @brief 绑定过程数据与偏移
     * @param domain_pd Domain 过程数据首地址
     * @param offsets PDO 偏移
     */
    DigitalIo(uint8_t* domain_pd, const Offsets& offsets)
        : domain_pd_(domain_pd), offsets_(offsets) {}

    /**
     * @brief 绑定 Domain 过程数据指针（activate 后）
     * @param domain_pd 可为 nullptr（随后读写返回 0 / no-op）
     */
    void setDomainData(uint8_t* domain_pd) noexcept { domain_pd_ = domain_pd; }
    /**
     * @brief 设置 PDO 偏移
     * @param offsets IO 偏移
     */
    void setOffsets(const Offsets& offsets) noexcept { offsets_ = offsets; }
    /**
     * @brief 从电机 Offsets 提取并设置 IO 偏移
     * @param m 电机 PDO 偏移
     */
    void setOffsetsFromMotor(const CiA402Motor::Offsets& m) noexcept {
        offsets_ = Offsets::fromMotorOffsets(m);
    }
    /** @brief 当前偏移 */
    const Offsets& offsets() const noexcept { return offsets_; }

    uint32_t readInputs() const noexcept override;
    uint32_t readOutputs() const noexcept override;
    void writeOutputs(uint32_t value) noexcept override;

    bool getInputBit(unsigned bit) const noexcept override;
    bool getOutputBit(unsigned bit) const noexcept override;
    void setOutputBit(unsigned bit, bool on) noexcept override;

private:
    uint8_t* domain_pd_ = nullptr;
    Offsets offsets_{};
};

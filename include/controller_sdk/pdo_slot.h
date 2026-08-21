#pragma once

/**
 * @file pdo_slot.h
 * @brief CiA402 / 数字 IO 过程数据语义槽（slot）
 *
 * PDO 映射仍走 IgH `ec_pdo_entry_reg_t`（按对象字典索引）。
 * 本表把「控制字 / 目标位置 / …」名字和 0x6040 / 0x607A 等索引绑在一起，
 * `SlaveConfigLoader` 用它填 `CiA402Motor::Offsets`，应用不要自己写死
 * `findOffsetPtr` 里的 case。
 *
 * ## RT / NRT
 * 查表是纯函数，RT 也可调；真正注册 PDO 只在 activate 前（NRT）。
 */

#include <cstdint>
#include <cstddef>
#include <cstring>

/** @brief 过程数据语义槽 */
enum class PdoSlot : uint8_t {
    Unknown = 0,
    Controlword,       ///< 0x6040
    Statusword,        ///< 0x6041
    TargetPosition,    ///< 0x607A
    TargetVelocity,    ///< 0x60FF
    TargetTorque,      ///< 0x6071
    OperationMode,     ///< 0x6060
    PositionActual,    ///< 0x6064
    VelocityActual,    ///< 0x606C
    TorqueActual,      ///< 0x6077
    ModeDisplay,       ///< 0x6061
    DigitalInputs,     ///< 0x60FD
    DigitalOutputs     ///< 0x60FE
};

/** @brief slot ↔ 对象字典 ↔ 常用名字 */
struct PdoSlotSpec {
    PdoSlot slot;          ///< 语义槽
    uint16_t index;        ///< OD 索引
    uint8_t subindex;      ///< 子索引；0xFF 表示 0 或 1 都认
    const char* name;      ///< 规范名（小写无空格）
};

inline const PdoSlotSpec* cia402PdoSlotTable(std::size_t& count) noexcept {
    static const PdoSlotSpec kSpecs[] = {
        {PdoSlot::Controlword,     0x6040, 0,    "controlword"},
        {PdoSlot::Statusword,      0x6041, 0,    "statusword"},
        {PdoSlot::TargetPosition,  0x607A, 0,    "targetposition"},
        {PdoSlot::TargetVelocity,  0x60FF, 0,    "targetvelocity"},
        {PdoSlot::TargetTorque,    0x6071, 0,    "targettorque"},
        {PdoSlot::OperationMode,   0x6060, 0,    "operationmode"},
        {PdoSlot::PositionActual,  0x6064, 0,    "positionactual"},
        {PdoSlot::VelocityActual,  0x606C, 0,    "velocityactual"},
        {PdoSlot::TorqueActual,    0x6077, 0,    "torqueactual"},
        {PdoSlot::ModeDisplay,     0x6061, 0,    "modedisplay"},
        {PdoSlot::DigitalInputs,   0x60FD, 0xFF, "digitalinputs"},
        {PdoSlot::DigitalOutputs,  0x60FE, 0xFF, "digitaloutputs"},
    };
    count = sizeof(kSpecs) / sizeof(kSpecs[0]);
    return kSpecs;
}

/** @brief 由对象字典索引/子索引查 slot；未登记返回 Unknown */
inline PdoSlot lookupPdoSlot(uint16_t index, uint8_t subindex) noexcept {
    std::size_t n = 0;
    const PdoSlotSpec* t = cia402PdoSlotTable(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (t[i].index != index) continue;
        if (t[i].subindex == 0xFF) {
            if (subindex == 0 || subindex == 1) return t[i].slot;
        } else if (t[i].subindex == subindex) {
            return t[i].slot;
        }
    }
    return PdoSlot::Unknown;
}

/** @brief 名字归一：小写、去掉空格/下划线/连字符 */
inline void normalizePdoSlotName(const char* in, char* out, std::size_t out_n) noexcept {
    if (!out || out_n == 0) return;
    std::size_t o = 0;
    if (in) {
        for (const char* p = in; *p && o + 1 < out_n; ++p) {
            const char c = *p;
            if (c == ' ' || c == '_' || c == '-') continue;
            out[o++] = (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
        }
    }
    out[o] = '\0';
}

/**
 * @brief 由 ESI/习惯名查 slot（"Control Word"、"controlword"、"0x6040" 均可）
 */
inline PdoSlot lookupPdoSlotByName(const char* name) noexcept {
    if (!name || !name[0]) return PdoSlot::Unknown;
    char norm[64];
    normalizePdoSlotName(name, norm, sizeof(norm));

    if (norm[0] == '0' && (norm[1] == 'x' || norm[1] == 'X')) {
        unsigned long idx = 0;
        for (const char* p = norm + 2; *p; ++p) {
            unsigned d = 0;
            if (*p >= '0' && *p <= '9') d = static_cast<unsigned>(*p - '0');
            else if (*p >= 'a' && *p <= 'f') d = static_cast<unsigned>(*p - 'a' + 10);
            else break;
            idx = (idx << 4) | d;
        }
        if (idx > 0 && idx <= 0xFFFFu)
            return lookupPdoSlot(static_cast<uint16_t>(idx), 0);
    }

    std::size_t n = 0;
    const PdoSlotSpec* t = cia402PdoSlotTable(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (std::strcmp(norm, t[i].name) == 0) return t[i].slot;
        // ESI 常写成 "Position actual value"；表名是 positionactual
        const std::size_t nl = std::strlen(t[i].name);
        if (nl >= 8 && std::strncmp(norm, t[i].name, nl) == 0 &&
            (norm[nl] == '\0' || std::strcmp(norm + nl, "value") == 0))
            return t[i].slot;
    }

    if (std::strcmp(norm, "modesofoperation") == 0 ||
        std::strcmp(norm, "modeofoperation") == 0)
        return PdoSlot::OperationMode;
    if (std::strcmp(norm, "modesofoperationdisplay") == 0 ||
        std::strcmp(norm, "modeofoperationdisplay") == 0)
        return PdoSlot::ModeDisplay;
    return PdoSlot::Unknown;
}

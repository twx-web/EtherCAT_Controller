#pragma once

/**
 * @file product_info.h
 * @brief 产品版本与开源许可信息（GPL-3.0）
 *
 * 本 SDK 以 GNU GPL v3 发布；与 IgH EtherCAT Master 一并分发时需遵守 GPL。
 * 无运行时激活码/加密狗——合规方式为保留版权声明与提供对应源码。
 */

#include <string>

#ifndef CONTROLLER_SDK_VERSION_STRING
#define CONTROLLER_SDK_VERSION_STRING "0.0.0"
#endif

namespace product_info {

/** @brief 产品显示名 */
inline const char* name() noexcept { return "Controller SDK"; }

/** @brief 语义化版本（CMake PROJECT_VERSION） */
inline const char* version() noexcept { return CONTROLLER_SDK_VERSION_STRING; }

/** @brief SPDX 许可证标识 */
inline const char* spdxLicense() noexcept { return "GPL-3.0-only"; }

/** @brief 许可证短标题 */
inline const char* licenseTitle() noexcept {
    return "GNU General Public License v3.0 only";
}

/** @brief 版权行（显示用） */
inline const char* copyright() noexcept {
    return "Copyright (C) Controller SDK contributors";
}

/** @brief 是否带 EtherCAT 后端（编译期） */
inline bool hasEthercat() noexcept {
#if defined(CONTROLLER_SDK_HAS_ETHERCAT) && CONTROLLER_SDK_HAS_ETHERCAT
    return true;
#else
    return false;
#endif
}

/** @brief 一行摘要，供 CLI / About */
inline std::string summaryLine() {
    std::string s = std::string(name()) + " " + version() + " (" + spdxLicense() + ")";
    s += hasEthercat() ? " [EtherCAT]" : " [core-only]";
    return s;
}

/** @brief 合规提示（中文） */
inline const char* complianceNoticeZh() noexcept {
    return "本软件按 GPL-3.0 许可发布。再分发时须保留许可证文本并提供对应源代码；"
           "与 IgH EtherCAT 组合分发时同样适用 GPL。详见仓库 LICENSE 文件。";
}

}  // namespace product_info

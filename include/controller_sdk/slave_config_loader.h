#pragma once

/**
 * @file slave_config_loader.h
 * @brief 从 EtherCAT XML 文件自动配置从站的工具类。
 *
 * 本类用于解析由 `ethercat xml -p <pos>` 生成的从站描述 XML 文件，
 * 自动完成 PDO 映射、同步管理器配置、PDO 注册表生成，
 * 并最终返回 CiA402Motor::Offsets，供电机对象使用。
 *
 * 设计原则：
 * - 用户只需提供 XML 文件路径、主站对象和从站位置。
 * - 内部自动处理所有 IgH 结构体填充与生命周期管理。
 * - 返回的 Offsets 可直接用于创建 CiA402Motor 实例。
 */

#include "EthercatMaster.h"
#include "CiA402Motor.h"

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <cstdint>

// 前向声明 tinyxml2 类型，避免头文件污染
namespace tinyxml2 {
    class XMLDocument;
    class XMLElement;
}

/**
 * @class SlaveConfigLoader
 * @brief 基于 XML 的从站配置加载器。
 *
 * 使用示例：
 * @code
 * EthercatMaster master;
 * // ... 初始化 master, createDomain, loadFromXml, activate ...
 * const auto* offsets = SlaveConfigLoader::loadFromXml(master, "slave0.xml", 0, 0);
 * if (offsets) {
 *     // activate 之后 offsets 才有有效值；务必持有指针，勿在 activate 前拷贝
 *     auto motor = std::make_unique<CiA402Motor>(sc, domainData, *offsets);
 * }
 * @endcode
 */
class SlaveConfigLoader {
public:
    // ==================== 从站配置信息结构体 ====================
    /** @brief PDO 条目信息 */
    struct PdoEntryInfo {
        uint16_t index = 0;         ///< 对象字典索引（如 0x6040）
        uint8_t subindex = 0;       ///< 子索引
        uint16_t bit_length = 0;    ///< 位长度
        std::string name;           ///< 条目名称（如 "Control word"）
    };

    /** @brief PDO 信息 */
    struct PdoInfo {
        uint16_t index = 0;         ///< PDO 索引（如 0x1600, 0x1A00）
        std::string name;           ///< PDO 名称
        std::string direction;      ///< "Rx" / "Tx"
        int sm = -1;                ///< ESI Sm 属性；未写为 -1
        bool assigned = false;      ///< 加载器实际会赋给 SM 的 PDO
        std::vector<uint16_t> excludes; ///< <Exclude> 互斥的其它 PDO 索引
        std::vector<PdoEntryInfo> entries;
    };

    /** @brief 从站配置信息（供用户查看，不配置从站） */
    struct SlaveInfo {
        uint32_t vendor_id = 0;
        uint32_t product_code = 0;
        std::string slave_name;

        struct SyncManagerInfo {
            int sm_index;
            std::string direction;
            int n_pdos;
        };
        std::vector<SyncManagerInfo> sync_managers;
        std::vector<PdoInfo> pdos;
    };

    /**
     * @brief 从 XML 文件解析并返回从站配置的关键信息（不配置从站）
     * @param xml_path XML 文件路径
     * @return 解析成功返回 SlaveInfo，失败返回 std::nullopt
     */
    static std::optional<SlaveInfo> getInfoFromXml(const std::string& xml_path);

    /**
     * @brief 从 XML 字符串内容解析并返回从站配置的关键信息（不配置从站）
     * @param xml_content XML 文件内容字符串
     * @return 解析成功返回 SlaveInfo，失败返回 std::nullopt
     */
    static std::optional<SlaveInfo> getInfoFromXmlContent(const std::string& xml_content);

    /**
     * @brief 从 ethercat xml 生成的 XML 文件加载从站配置。
     *
     * 本函数会解析 XML 中的 Vendor ID、Product Code、SM 设置、
     * RxPDO/TxPDO 条目，自动构建 ec_sync_info_t、ec_pdo_info_t、
     * ec_pdo_entry_reg_t 等数组，并调用主站接口完成从站添加与 PDO 注册。
     *
     * PDO 选择（兼容两类 ESI）：
     * - 叠加型（GSHD）：多路 PDO 都带 Sm="2"/"3" 且无互斥，全部赋给对应 SM。
     * - 互斥型：<Exclude> 标明不能同时选；优先只取带 Sm= 的默认 PDO
     *   （例如 1701+1B01），避免 1600/1601 里重复的 6040 被一起映射。
     * - XML 都没有 Sm 属性时，按文档顺序贪心选取互不 Exclude、对象不重叠的集合。
     *
     * 输入参数：
     * @param master   EtherCAT 主站对象（已初始化，domain 已创建）。
     * @param xml_path XML 文件路径（可由 ethercat xml -p <pos> 生成）。
     * @param alias    从站别名，默认 0。
     * @param position 从站物理位置，默认 0。
     *
     * 输出：
     * @return 成功时返回指向内部静态存储中 Offsets 的指针；失败返回 nullptr。
     *
     * 说明：
     * - 解析的 XML 必须是由 `ethercat xml` 生成的格式。
     * - 返回的是指针而非值拷贝：IgH 在 activate() 后才写入真实偏移。
     * - 必须在 master.activate() 之后再用 *offsets 构造电机；activate 前拷贝无效。
     * - 指针在进程生命周期内有效（内部静态容器持有）。
     */
    static const CiA402Motor::Offsets*
    loadFromXml(EthercatMaster& master,
                const std::string& xml_path,
                uint16_t alias = 0,
                uint16_t position = 0);

    /**
     * @brief 自动扫描并配置所有 EtherCAT 从站。
     *
     * 内部调用 `ethercat xml` 命令获取每个从站的描述信息，
     * 然后自动调用 loadFromXml 完成配置。
     *
     * 输入参数：
     * @param master    EtherCAT 主站对象（已初始化，domain 已创建）。
     * @param max_slaves 最大扫描从站数（默认 16）。
     *
     * 输出：
     * @return 按从站添加顺序排列的 Offsets 指针列表（activate 后可读）。
     *
     * 说明：
     * - 要求主站尚未激活，且 `ethercat` 命令行工具已安装且可用。
     * - 从站必须上电并连接至总线，否则将被跳过。
     */
    static std::vector<const CiA402Motor::Offsets*>
    autoConfigureAllSlaves(EthercatMaster& master, uint16_t max_slaves = 16);

private:
    /**
     * @struct SlaveConfigData
     * @brief 从站配置的内部数据容器。
     *
     * 用于持有解析出的所有 IgH 结构体数组，保证生命周期足够长。
     */
    struct SlaveConfigData {
        uint32_t vendor_id = 0;               ///< 厂商 ID
        uint32_t product_code = 0;            ///< 产品代码

        std::vector<ec_sync_info_t> syncs;          ///< 同步管理器配置
        std::vector<ec_pdo_entry_info_t> rx_entries; ///< RxPDO 条目
        std::vector<std::string> rx_entry_names;     ///< 与 rx_entries 平行的 ESI Name
        std::vector<ec_pdo_entry_info_t> tx_entries; ///< TxPDO 条目
        std::vector<std::string> tx_entry_names;     ///< 与 tx_entries 平行的 ESI Name
        std::vector<ec_pdo_info_t> pdos;             ///< PDO 信息
        std::vector<ec_pdo_entry_reg_t> regs;        ///< PDO 注册表

        CiA402Motor::Offsets offsets;                ///< 最终偏移量
    };

    /**
     * @brief 解析同步管理器 (SM) 配置。
     *
     * 输入参数：
     * @param deviceElem XML 中的 <Device> 元素指针。
     * @param syncsOut   输出解析出的 sync 数组（至少包含 SM0~SM3）。
     *
     * 输出：
     * @return true  解析成功。
     * @return false 解析失败（如 SM 数量不足）。
     */
    static bool parseSyncManagers(tinyxml2::XMLElement* deviceElem,
                                  std::vector<ec_sync_info_t>& syncsOut);

    /**
     * @brief 解析单个 PDO (RxPdo 或 TxPdo) 中的条目。
     *
     * 输入参数：
     * @param pdoElem    XML 中的 <RxPdo> 或 <TxPdo> 元素指针。
     * @param entriesOut 输出解析出的条目列表。
     *
     * 输出：
     * @return true  至少解析到一个有效条目。
     * @return false 未解析到任何条目。
     */
    static bool parsePdoEntries(tinyxml2::XMLElement* pdoElem,
                                std::vector<ec_pdo_entry_info_t>& entriesOut,
                                std::vector<std::string>& namesOut);

    struct XmlPdo {
        uint16_t index = 0;
        int sm = -1;
        std::string name;
        std::vector<uint16_t> excludes;
        std::vector<ec_pdo_entry_info_t> entries;
        std::vector<std::string> entry_names; ///< 与 entries 平行的 <Name>
    };

    static uint32_t parseHexOrDec(const char* text);
    static bool parseXmlPdo(tinyxml2::XMLElement* pdoElem, XmlPdo& out);
    static std::vector<XmlPdo> collectXmlPdos(tinyxml2::XMLElement* deviceElem,
                                              const char* tag);
    static bool xmlPdoConflicts(const XmlPdo& a, const XmlPdo& b);
    static std::vector<XmlPdo> selectAssignedPdos(const std::vector<XmlPdo>& all,
                                                  int expected_sm);

    /**
     * @brief 根据对象字典索引/子索引，获取 Offsets 中对应字段的指针。
     *        语义表见 `pdo_slot.h`（`lookupPdoSlot`）。
     *
     * 输入参数：
     * @param offsets   Offsets 结构体引用。
     * @param index     对象字典索引。
     * @param subindex  子索引。
     *
     * 输出：
     * @return 指向 Offsets 中对应字段的指针，若未匹配则返回 nullptr。
     */
    static unsigned int* findOffsetPtr(CiA402Motor::Offsets& offsets,
                                       uint16_t index, uint8_t subindex);

    /**
     * @brief 辅助函数：将 XML 元素的文本内容解析为 uint32_t。
     *
     * 自动处理 "#x" 前缀的十六进制数值。
     *
     * 输入参数：
     * @param elem XML 元素指针。
     *
     * 输出：
     * @return 解析后的整数值，若元素为空则返回 0。
     */
    static uint32_t parseEcValue(tinyxml2::XMLElement* elem);
};
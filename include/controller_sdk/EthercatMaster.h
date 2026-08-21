#pragma once

/**
 * @file EthercatMaster.h
 * @brief EtherCAT 主站封装类（工业级长期稳定底座）
 *
 * 设计原则：
 * 1. 仅封装主站生命周期、Domain、PDO 注册、SDO、DC 同步等通信能力
 * 2. 不包含任何电机/轴/轨迹等高层逻辑
 * 3. 提供原始句柄接口以便高级用户扩展
 * 4. 所有状态查询和错误信息均已线程安全保护
 * 5. 日志与错误可通过回调函数接入客户系统
 */

#include <ecrt.h>
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <mutex>
#include <functional>

/**
 * @class EthercatMaster
 * @brief IgH EtherCAT 主站封装类
 */
class EthercatMaster {
public:
    /** @brief 从站配置结构 */
    struct SlaveConfig {
        uint16_t alias = 0;            ///< 从站别名
        uint16_t position = 0;         ///< 从站物理位置
        uint32_t vendor_id = 0;        ///< 厂商 ID
        uint32_t product_code = 0;     ///< 产品代码
        ec_sync_info_t* syncs = nullptr; ///< PDO 同步管理器配置表
    };

    /** @brief Domain 工作状态 */
    struct DomainState {
        unsigned int working_counter = 0; ///< 实际工作计数器
        unsigned int expected_working_counter = 0; ///< 期望 WKC（首次 COMPLETE 后闩锁；0=未知）
        ec_wc_state_t wc_state = EC_WC_ZERO; ///< 工作计数器状态
    };

    /** @brief Master 工作状态 */
    struct MasterState {
        unsigned int slaves_responding = 0; ///< 响应从站数量
        unsigned int al_states = 0;         ///< 从站 AL 状态组合值
        bool link_up = false;               ///< 网卡链路是否正常
    };

    /** @brief 主站静态信息（封装 IgH 原生结构） */
    struct MasterInfo {
        unsigned int master_index = 0;   ///< 主站索引
        unsigned int slave_count = 0;    ///< 总从站数量（含未响应）
        uint64_t min_timestamp_ns = 0;   ///< 最小时间戳（纳秒）
        uint64_t max_timestamp_ns = 0;   ///< 最大时间戳（纳秒）
        bool link_up = false;            ///< 以太网链路状态
    };

    /** @brief 从站静态信息（封装 IgH 原生结构） */
    struct SlaveInfo {
        uint16_t position = 0;           ///< 从站物理位置
        uint32_t vendor_id = 0;          ///< 厂商 ID
        uint32_t product_code = 0;       ///< 产品代码
        uint32_t revision_no = 0;        ///< 版本号
        uint32_t serial_no = 0;          ///< 序列号
        uint8_t al_state = 0;            ///< AL 状态（EC_STATE_*）
        bool link_up = false;            ///< 链路状态
        bool current_on_ethercat = false;///< 是否在 EtherCAT 总线上
        std::string name;                ///< 从站名称
    };

    /** @brief 日志级别 */
    enum class LogLevel { DEBUG, INFO, WARN, ERROR };

    /** @brief 日志回调类型 */
    using LogCallback = std::function<void(LogLevel level, const std::string& msg)>;

    /** @brief 错误回调类型 */
    using ErrorCallback = std::function<void(const std::string& error)>;

public:
    /** @brief 构造函数 */
    EthercatMaster();

    /** @brief 析构函数，自动释放资源 */
    ~EthercatMaster();

    // 禁止拷贝
    EthercatMaster(const EthercatMaster&) = delete;
    EthercatMaster& operator=(const EthercatMaster&) = delete;

    // ---------- 生命周期 ----------

    /**
     * @brief 初始化 EtherCAT 主站
     * @param master_index 主站索引（默认 0）
     * @return true 成功，false 失败
     */
    bool init(unsigned int master_index = 0);

    /** @brief 释放所有资源（主站、domain、从站配置） */
    void deinit();

    // ---------- Domain ----------

    /**
     * @brief 创建 Process Data Domain
     * @return true 成功，false 失败
     */
    bool createDomain();

    // ---------- 从站配置 ----------

    /**
     * @brief 添加一个从站配置
     * @param config 从站配置结构
     * @return 成功返回从站配置句柄，失败返回 nullptr
     */
    ec_slave_config_t* addSlaveConfig(const SlaveConfig& config);

    /**
     * @brief 注册 PDO 入口偏移量表
     * @param regs 以空结构结尾的注册表数组
     * @return true 成功，false 失败
     */
    bool registerPdoEntries(const ec_pdo_entry_reg_t* regs);

    // ---------- 激活与运行 ----------

    /**
     * @brief 激活主站（启动总线通信）
     * @return true 成功，false 失败
     */
    bool activate();

    /**
     * @brief 执行一个最小 EtherCAT 周期（接收 -> 处理 -> 更新状态 -> 队列 -> 发送）
     *        通常在实时线程中周期性调用
     */
    void cycleOnce();

    // ---------- 分布式时钟 ----------

    /**
     * @brief 为指定从站配置 DC 参数
     * @param slave_config 从站配置句柄
     * @param assign_activate 分配激活字
     * @param sync0_cycle_ns Sync0 周期（纳秒）
     * @param sync0_shift_ns Sync0 偏移（纳秒）
     * @param sync1_cycle_ns Sync1 周期（纳秒，可选）
     * @param sync1_shift_ns Sync1 偏移（纳秒，可选）
     * @return true 成功，false 失败
     */
    bool configureDistributedClock(
        ec_slave_config_t* slave_config,
        uint16_t assign_activate,
        uint32_t sync0_cycle_ns,
        int32_t sync0_shift_ns,
        uint32_t sync1_cycle_ns = 0,
        int32_t sync1_shift_ns = 0
    );

    /**
     * @brief 为当前已添加的全部从站配置 DC（须在 activate 之前）
     * @param assign_activate 0 表示不改写（保留从站默认）；常见 SYNC0 为 0x0300
     * @param sync0_cycle_ns Sync0 周期 [ns]，一般等于控制周期
     * @param sync0_shift_ns Sync0 偏移 [ns]
     */
    bool configureAllSlavesDc(uint16_t assign_activate, uint32_t sync0_cycle_ns,
                              int32_t sync0_shift_ns = 0);

    /**
     * @brief activate 前检查 DC 组合：enable_dc 且 assign=0 时打 WARN
     *
     * 主站仍会 `syncDistributedClocks`，但从站未写 AssignActivate（无 SYNC0）。
     * 要 DC：`dc_assign_activate=768`（0x0300）；不要 DC：`enable_dc=false`。
     */
    void warnDcConfig(bool enable_dc, uint16_t assign_activate);

    /**
     * @brief 最近一次 DC 同步监视上界 [ns]（`0x092c` 广播读，RT 每拍更新）
     * @return 各从站系统时间差的上估计；未开 DC 或尚未交换为 0
     */
    uint32_t dcDiffNs() const noexcept {
        return dc_diff_ns_.load(std::memory_order_relaxed);
    }

    /** @brief 0x1C32/0x1C33 同步诊断（NRT SDO，禁止 RT） */
    struct SmSyncDiag {
        bool valid = false;                 ///< 至少读到一项
        bool sm2_sync_error = false;        ///< 0x1C32:32
        bool sm3_sync_error = false;        ///< 0x1C33:32
        uint16_t sm2_cycle_exceeded = 0;    ///< 0x1C32:12 Cycle exceeded
        uint16_t sm3_cycle_exceeded = 0;    ///< 0x1C33:12
        uint16_t sm2_sm_event_missed = 0;   ///< 0x1C32:11
        uint16_t sm3_sm_event_missed = 0;   ///< 0x1C33:11
    };

    /**
     * @brief NRT：读指定从站 SM2/SM3 同步误差对象
     * @param slave_position 物理位置
     * @param[out] out 诊断
     * @param timeout_ms 单个 SDO 超时
     * @return 主站可用且至少成功一项为 true
     */
    bool readSmSyncDiag(uint16_t slave_position, SmSyncDiag& out,
                        uint32_t timeout_ms = 200);

    /**
     * @brief 由超时毫秒计算 IgH 看门狗 divider/intervals（基时基 100µs）
     * @return false 表示 timeout_ms==0（使用从站默认，不写寄存器）
     */
    static bool computeWatchdogParams(uint32_t timeout_ms,
                                      uint16_t& divider,
                                      uint16_t& intervals) noexcept;

    /**
     * @brief 配置单从站过程数据看门狗（须在 activate 之前，NRT）
     * @param timeout_ms 超时；0 表示不改写（保留从站默认）
     */
    bool configureSlaveWatchdog(ec_slave_config_t* sc, uint32_t timeout_ms);

    /**
     * @brief 为当前已添加的全部从站配置看门狗（须在 activate 之前）
     */
    bool configureAllSlavesWatchdog(uint32_t timeout_ms);

    /**
     * @brief 由周期数计算 IgH 看门狗参数（基时基 100µs）
     * @param cycles 超时周期数；0 表示不改写
     * @param cycle_us 控制周期（微秒），1 kHz 为 1000
     */
    static bool computeWatchdogParamsFromCycles(uint32_t cycles,
                                                uint32_t cycle_us,
                                                uint16_t& divider,
                                                uint16_t& intervals) noexcept;

    /**
     * @brief 按控制周期数配置全部从站看门狗（须在 activate 之前）
     */
    bool configureAllSlavesWatchdogCycles(uint32_t cycles, uint32_t cycle_us);

    /**
     * @brief 手动设定期望 WKC（NRT；0=未知）。首次 WC_COMPLETE 后会自动闩锁实际值。
     */
    void setExpectedWorkingCounter(unsigned int expected);

    /**
     * @brief 设置应用时间（DC 同步基准，纳秒）
     * @param app_time_ns 应用时间，通常取 CLOCK_MONOTONIC 纳秒
     */
    void setApplicationTime(uint64_t app_time_ns);

    /**
     * @brief 同步参考时钟及所有从站时钟
     *
     * 调用前应先 setApplicationTime()；否则仅做 slave 时钟同步，漂移补偿不完整。
     */
    void syncDistributedClocks();

    /**
     * @brief 标准 DC 同步序列：application_time → sync_reference → sync_slaves
     * @param app_time_ns 应用时间（纳秒）
     */
    void syncDistributedClocks(uint64_t app_time_ns);

    // ---------- SDO 操作（NRT；无请求缓存，推荐优先用 SdoClient）----------

    /**
     * @brief SDO 下载（写对象字典）
     * @param slave_position 从站物理位置
     * @param index 对象索引
     * @param subindex 子索引
     * @param data 数据指针
     * @param data_size 数据字节数
     * @param abort_code 输出 SDO 中止码（可选）
     * @return true 成功，false 失败
     * @note 每次创建临时请求；频繁访问请使用 `SdoClient`（见 sdo_client.h）。
     */
    bool sdoDownload(
        uint16_t slave_position,
        uint16_t index,
        uint8_t subindex,
        const void* data,
        size_t data_size,
        uint32_t* abort_code = nullptr
    );

    /**
     * @brief SDO 上传（读对象字典）
     * @param slave_position 从站物理位置
     * @param index 对象索引
     * @param subindex 子索引
     * @param buffer 接收缓冲区
     * @param buffer_size 缓冲区大小
     * @param result_size 实际读取字节数（可选）
     * @param abort_code 输出 SDO 中止码（可选）
     * @return true 成功，false 失败
     */
    bool sdoUpload(
        uint16_t slave_position,
        uint16_t index,
        uint8_t subindex,
        void* buffer,
        size_t buffer_size,
        size_t* result_size = nullptr,
        uint32_t* abort_code = nullptr
    );

    // ---------- 状态与诊断 ----------

    /**
     * @brief 获取 Domain 过程数据指针（供电机类直接读写）
     * @return 过程数据内存首地址
     */
    uint8_t* domainData() const;

    /**
     * @brief 获取 Domain 状态（线程安全）
     * @return Domain 状态副本
     */
    DomainState domainState() const;

    /**
     * @brief 获取 Master 状态（线程安全）
     * @return Master 状态副本
     */
    MasterState masterState() const;

    /**
     * @brief 检查总线是否正常工作（链路通且至少有一个从站响应）
     * @return true 正常，false 异常
     *
     * @note 这是弱检查。运动控制请用 isDomainWcComplete() /
     *       areConfiguredSlavesOperational() / isCommunicationOk()。
     */
    bool isOperational() const;

    /**
     * @brief Domain 工作计数器是否完整（EC_WC_COMPLETE）
     * @note 需在 processDomain + updateDomainState 之后调用
     */
    bool isDomainWcComplete() const;

    /**
     * @brief 已配置从站是否均 online 且 operational (OP)
     */
    bool areConfiguredSlavesOperational() const;

    /**
     * @brief 通信是否允许运动：链路 + WC 完整 + 从站 OP
     */
    bool isCommunicationOk() const;

    /**
     * @brief 周期前半段：application_time → receive → process → 状态 → DC sync
     * @param app_time_ns 应用时间（纳秒）；为 0 时自动取 CLOCK_MONOTONIC
     * @param enable_dc 是否执行 DC 同步
     */
    void cycleBegin(uint64_t app_time_ns = 0, bool enable_dc = true);

    /**
     * @brief 周期后半段：queue → send
     */
    void cycleEnd();

    /**
     * @brief 获取最近一次错误信息
     * @return 错误描述字符串
     */
    std::string lastError() const;

    // ---------- 从站信息查询 ----------

    /**
     * @brief 获取主站静态信息（封装后，无需包含 ecrt.h）
     * @return MasterInfo 结构体，失败时字段为默认值（0/false）
     */
    MasterInfo getMasterInfo() const;

    /**
     * @brief 获取指定从站的静态信息（封装后，无需包含 ecrt.h）
     * @param slave_position 从站物理位置
     * @return SlaveInfo 结构体，失败时字段为默认值（0/false）
     */
    SlaveInfo getSlaveInfo(uint16_t slave_position) const;

    // ---------- 原始句柄（高级扩展用）----------

    /** @brief 获取原始 master 指针 */
    ec_master_t* nativeMaster() const;
    /** @brief 获取原始 domain 指针 */
    ec_domain_t* nativeDomain() const;
    /**
     * @brief 获取指定索引的从站配置句柄
     * @param index 添加顺序索引
     * @return 从站配置句柄，索引无效返回 nullptr
     */
    ec_slave_config_t* nativeSlaveConfig(size_t index) const;
    /** @brief 获取已添加的从站数量 */
    size_t slaveConfigCount() const;

    // ---------- 回调设置 ----------

    /**
     * @brief 设置日志回调
     * @param callback 用户提供的日志处理函数
     */
    void setLogCallback(LogCallback callback);

    /**
     * @brief 设置错误回调
     * @param callback 用户提供的错误处理函数
     */
    void setErrorCallback(ErrorCallback callback);

    /** 从网卡接收帧 */
    void receive();
    /** 处理 Domain 的 process data */
    void processDomain();
    /** 将 Domain 数据加入发送队列 */
    void queueDomain();
    /** 发送 EtherCAT 帧 */
    void send();

    /** 更新 Domain 状态缓存 */
    void updateDomainState();
    /** 更新 Master 状态缓存 */
    void updateMasterState();

private:
    // 修改点1：setError 改为 const，last_error_ 改为 mutable
    void setError(const std::string& error) const;
    void log(LogLevel level, const std::string& msg) const;

    ec_master_t* master_ = nullptr;
    ec_domain_t* domain_ = nullptr;
    uint8_t* domain_pd_ = nullptr;

    std::vector<ec_slave_config_t*> slave_configs_;          ///< 按添加顺序保存的从站配置
    std::map<uint16_t, ec_slave_config_t*> pos_to_config_;   ///< 物理位置到配置句柄的映射

    DomainState domain_state_;
    MasterState master_state_;
    std::atomic<uint32_t> dc_diff_ns_{0};
    bool dc_cycle_enabled_ = false;
    bool dc_monitor_queued_ = false;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> activated_{false};

    mutable std::mutex state_mutex_;    ///< 保护 domain/master 状态
    mutable std::mutex error_mutex_;    ///< 保护错误信息
    mutable std::string last_error_;    ///< 修改为 mutable，允许在 const 函数中修改

    LogCallback log_callback_;
    ErrorCallback error_callback_;
};
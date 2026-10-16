/**
 * @file    eth.cpp
 * @author  syhanjin
 * @date    2026-10-08
 * @brief   Ethernet BSP 启动实现（见 eth.hpp 的接线、线程与计数说明）。
 *
 * 与 LwipPlatform 的职责边界：本文件拥有 ETH 句柄关联的静态对象（PHY、平台、回调）与
 * 启动次序；平台拥有链路线程并负责 MAC 停启与 netif link 切换。TX 完成回调保留
 * CubeMX 的 TxPktSemaphore 通知并转给平台回收；计数只表示 MAC 完成回调活动，
 * 可能合并多个完成，不能据此认定某个应用 UDP 报文已送达。
 */
#include "eth.hpp"

#include "DP83822Phy.hpp"
#include "IPhy.hpp"
#include "LwipPlatform.hpp"
#include "gpio_driver.h"
#include "lwip.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"

#include "cmsis_os2.h"

#include <cstdint>

namespace
{
using bsp::ethernet_phy::DP83822Phy;
using bsp::ethernet_phy::LwipPlatform;
using bsp::ethernet_phy::PhyLinkConfig;
using bsp::ethernet_phy::PhyLinkMode;
using bsp::ethernet_phy::PollResult;

/** @brief 原理图固定的 PHY MDIO 地址。 */
constexpr std::uint32_t kPhyAddress = 1U;
/** @brief 通告全部 10/100 Mbps 模式，交给 PHY 自动协商（含 Auto-MDIX）。 */
constexpr PhyLinkConfig kLinkConfig{true, PhyLinkMode::All10_100};
/** @brief 等待 MX_LWIP_Init 完成 netif 建立的上限（tick = ms）；超时不阻塞整机测试。 */
constexpr std::uint32_t kLwipWaitTimeoutMs = 3000U;

GPIO_t reset_n{ETH_RESET_N_GPIO_Port, ETH_RESET_N_Pin};
GPIO_t int_pwdn_n{ETH_INP_PWDN_N_GPIO_Port, ETH_INP_PWDN_N_Pin};

/* PB10 = PHY INT/PWDN_N 为低有效中断输入。CubeMX 生成下降沿 EXTI、NVIC priority 5
 * 与 EXTI15_10_IRQHandler；UserCode/eth/phy_int.cpp 只提供唯一 HAL_GPIO_EXTI_Callback
 * -> bsp::gpio::DispatchExtiInterrupt 桥接。有效 INT 参数使 DP83822Phy 使用中断模式：
 * ISR 仅唤醒 LwipPlatform 的 EthLink 线程，MDIO 确认与链路处理全由既有包负责。
 * 不在 UserCode 追加 PHY 轮询、NVIC 配置或寄存器访问。 */
DP83822Phy  phy{heth, kPhyAddress, reset_n, int_pwdn_n};
LwipPlatform platform{heth, phy};

/* 以下状态由 EthLink/ETH ISR 写入，由测试线程轮询读取；volatile 使 SWD 与轮询可见，
   计数器为单写者自增，读方接受 live snapshot 语义（见 UserCode/test_common.hpp）。 */
volatile std::uint32_t tx_complete_count   = 0U;
volatile std::uint32_t platform_error_code = 0U;
volatile bool          platform_failed     = false;

bool    init_attempted = false;
EthInit init_result    = EthInit::LwipTimeout;

/**
 * @brief MAC TX 完成回调：记录实际完成事件，并保留平台与 CubeMX 的通知。
 *
 * 回调并非逐应用报文完成通知，可能合并完成；该计数仅作整机链路活动旁证。
 */
void onTxComplete(ETH_HandleTypeDef* handle) noexcept
{
    tx_complete_count = tx_complete_count + 1U;
    HAL_ETH_TxCpltCallback(handle); /* 保留 CubeMX 原生 TxPktSemaphore 信号链。 */
    platform.notifyTxComplete();
}

/** @brief EthLink 线程致命错误回调：只记录，不停机也不重试。 */
void onPlatformError(const PollResult error) noexcept
{
    platform_error_code = static_cast<std::uint32_t>(error);
    platform_failed     = true;
}

/**
 * @brief 等待 netif 建立并由 MX_LWIP_Init 置 up；超时返回 false。
 *
 * netif_add 内部调用 ethernetif_init → low_level_init → HAL_ETH_Init，故 netif 可见即表示
 * HAL 已处于 READY，可注册动态回调。defaultTask 优先级低于本线程，必须 osDelay 让其运行。
 */
bool waitForNetifUp() noexcept
{
    const std::uint32_t start = osKernelGetTickCount();
    while ((netif_default == nullptr) || (netif_is_up(netif_default) == 0U))
    {
        if (static_cast<std::uint32_t>(osKernelGetTickCount() - start) >= kLwipWaitTimeoutMs)
            return false;
        osDelay(1U);
    }
    return true;
}

} // namespace

EthInit ETH_Init()
{
    if (init_attempted)
        return init_result; /* 幂等：回调与链路线程都已安装，不重复启动。 */

    if (!waitForNetifUp())
        return init_result = EthInit::LwipTimeout; /* 未安装任何东西，允许稍后重试。 */

    if (HAL_ETH_RegisterCallback(&heth, HAL_ETH_TX_COMPLETE_CB_ID, &onTxComplete) != HAL_OK)
        return init_result = EthInit::TxCallbackError; /* 未安装回调，允许重试。 */

    if (!platform.start(*netif_default, kLinkConfig, &onPlatformError))
    {
        init_attempted = true; /* 平台已记下本次尝试，不能再次 start。 */
        return init_result = EthInit::LinkThreadError;
    }

    init_attempted = true;
    return init_result = EthInit::Ok;
}

const char* eth_init_name(const EthInit result)
{
    switch (result)
    {
    case EthInit::Ok: return "OK";
    case EthInit::LwipTimeout: return "LWIP_TIMEOUT";
    case EthInit::TxCallbackError: return "TX_CB_ERROR";
    case EthInit::LinkThreadError: return "LINK_THREAD_ERROR";
    case EthInit::PlatformError: return "PLATFORM_ERROR";
    }
    return "UNKNOWN";
}

bool ETH_PlatformFailed()
{
    return platform_failed;
}

std::uint32_t ETH_PlatformError()
{
    return platform_error_code;
}

const char* eth_platform_error_name(const std::uint32_t code)
{
    switch (static_cast<PollResult>(code))
    {
    case PollResult::Ok: return "NONE";
    case PollResult::PhyReadError: return "PHY_READ_ERROR";
    case PollResult::MacStopError: return "MAC_STOP_ERROR";
    case PollResult::MacStartError: return "MAC_START_ERROR";
    case PollResult::NotInitialized: return "NOT_INITIALIZED";
    case PollResult::InvalidPhyState: return "INVALID_PHY_STATE";
    case PollResult::PhyStartError: return "PHY_START_ERROR";
    case PollResult::PhyInterruptError: return "PHY_INTERRUPT_ERROR";
    case PollResult::ThreadWaitError: return "THREAD_WAIT_ERROR";
    }
    return "UNKNOWN";
}

std::uint32_t ETH_TxCompleteCount()
{
    return tx_complete_count;
}


bool ETH_LinkUp()
{
    if (netif_default == nullptr)
        return false;

    /* netif link 标志由 EthLink 线程在 core lock 内改写，读取同样取锁，避免读到切换中的状态。 */
    LOCK_TCPIP_CORE();
    const bool up = (netif_is_link_up(netif_default) != 0U);
    UNLOCK_TCPIP_CORE();
    return up;
}

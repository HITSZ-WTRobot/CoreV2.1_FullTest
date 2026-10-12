/**
 * @file    eth.cpp
 * @author  syhanjin
 * @date    2026-10-08
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "eth.hpp"

#include "DP83822Phy.hpp"
#include "IPhy.hpp"
#include "LwipPlatform.hpp"
#include "gpio_driver.h"
#include "lwip.h"

namespace
{
using bsp::ethernet_phy::LwipPlatform;
using bsp::ethernet_phy::DP83822Phy;
using bsp::ethernet_phy::PhyLinkConfig;
using bsp::ethernet_phy::PhyLinkMode;
using bsp::ethernet_phy::PollResult;

constexpr uint32_t PhyAddress = 1U;
constexpr PhyLinkConfig LinkConfig {true, PhyLinkMode::All10_100};

GPIO_t ResetN {ETH_RESET_N_GPIO_Port, ETH_RESET_N_Pin};
GPIO_t InpN {ETH_INP_PWDN_N_GPIO_Port, ETH_INP_PWDN_N_Pin};

DP83822Phy phy{heth, PhyAddress, ResetN, InpN};
LwipPlatform platform {heth, phy};

/**
 * @brief 等待 MX_LWIP_Init 完成协议栈与 netif 建立。
 *
 * netif_add 内部调用 ethernetif_init → low_level_init → HAL_ETH_Init，因此 netif 可见即表示
 * HAL 已处于 READY，可注册动态回调；netif_set_up 在 netif_add 之后调用。
 * 本任务优先级高于 defaultTask，必须让出 CPU 让其运行，不能忙等。
 */
void waitForLwipInit()
{
    while ((netif_default == nullptr) || (netif_is_up(netif_default) == 0U))
        osDelay(1U);
}

}

void ETH_Init()
{
    /* 协议栈与 MAC 就绪后再接线：HAL 动态回调只允许在 READY 状态注册。 */
    waitForLwipInit();

    /* 保留 Cube 默认的 TxPktSemaphore 通知，再通知平台回收已完成 TX。 */
    if (HAL_ETH_RegisterCallback(&heth,
                                 HAL_ETH_TX_COMPLETE_CB_ID,
                                 [](ETH_HandleTypeDef* handle) {
                                     HAL_ETH_TxCpltCallback(handle);
                                     platform.notifyTxComplete();
                                 }) != HAL_OK)
        Error_Handler();

    /* 创建平台自有的 EthLink 线程；PHY 初始化在线程内异步进行。 */
    if (!platform.start(*netif_default, LinkConfig, [](PollResult) noexcept { Error_Handler(); }))
        Error_Handler();
}
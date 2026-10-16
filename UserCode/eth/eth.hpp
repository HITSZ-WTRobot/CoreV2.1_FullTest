/**
 * @file    eth.hpp
 * @author  syhanjin
 * @date    2026-10-08
 * @brief   Ethernet BSP 启动入口：RMII + DP83822 PHY + LwIP netif + MAC 链路线程。
 *
 * 本文件只负责“把 MAC 与 PHY 拉起来”，不产生任何业务流量；UDP 收发与整板
 * 测试协议见 UserCode/ethernet_test.hpp。
 *
 * 固定硬件接线（来自 CubeMX 工程与 Core/Src/gpio.c，Board: STM32H723VET6）：
 *   RMII:  PA1 REF_CLK, PA2 MDIO, PA7 CRS_DV, PB11 TX_EN, PB12 TXD0, PB13 TXD1,
 *          PC1 MDC, PC4 RXD0, PC5 RXD1；MDIO 依赖板上外部上拉。
 *   PHY:   DP83822，MDIO 地址 1；PB14 = RESET_N（低有效）。
 *          PB10 = PHY INT/PWDN_N 低有效中断输入，上拉、下降沿 EXTI15_10、NVIC priority 5。
 *          GPIO/EXTI IRQ 由 CubeMX 生成；USER_PHY 不生成 ETH MSP，eth/msp.cpp 保留唯一
 *          HAL-only 板级 glue；phy_int.cpp 只作 HAL_GPIO_EXTI_Callback -> GPIO_Driver 桥接。
 *          传有效 INT 给 DP83822Phy，ISR 仅唤醒 LwipPlatform；MDIO 确认和链路切换由包负责。
 *   IP:    192.168.1.48/24，GW 192.168.1.1（LWIP/App/lwip.c 的 MX_LWIP_Init）。
 *   IRQ:   ETH_IRQn 优先级 5（UserCode/eth/msp.cpp 的 HAL MSP），与 FreeRTOS ISR API 兼容。
 *
 * 启动次序与线程：
 *   - MX_LWIP_Init() 由 CubeMX 生成的 defaultTask 执行（netif_add → HAL_ETH_Init → netif_set_up）；
 *     ETH_Init() 等待 netif 就绪后才注册回调并创建 EthLink 线程，因此必须从线程上下文调用，
 *     且调用期间会 osDelay 让出 CPU（不得在中断或调度器启动前调用）。
 *   - PHY 复位/配置、链路事件确认（由 PHY EXTI 唤醒）与链路查询都发生在 LwipPlatform 自有的
 *     EthLink 线程（osPriorityBelowNormal）：ISR 内只置线程标志，本文件不轮询 MDIO。
 *   - 致命链路错误不再停机：LwipPlatform::fail() 的错误码被记录，供测试模块上报 FAIL，
 *     使其余外设测试继续运行（无网线/首次配置不会挂死整机测试）。
 *
 * 计数语义：ETH_TxCompleteCount() 统计 MAC 实际 TX 完成回调事件（可能合并完成），
 * 含 ARP/ICMP，不能逐报文关联；仅为 Status::tx 整机旁证，不是应用送达证明。
 */
#pragma once

#include <cstdint>

/** @brief ETH_Init() 的启动结果；失败时不重试硬件，只如实上报。 */
enum class EthInit : std::uint32_t
{
    Ok = 0,            ///< netif 就绪、TX 回调已注册、EthLink 线程已创建。
    LwipTimeout,       ///< 等待 netif up 超时（MX_LWIP_Init 未跑完或 HAL_ETH_Init 失败）。
    TxCallbackError,   ///< HAL_ETH_RegisterCallback(TX_COMPLETE) 失败。
    LinkThreadError,   ///< LwipPlatform::start() 失败（重复调用或线程创建失败）。
    PlatformError,     ///< EthLink 线程终止；错误码见 ETH_PlatformError()（运行时可能后才出现）。
};

/**
 * @brief 启动以太网 BSP（幂等）；只能从线程上下文调用一次（重复调用返回首次结果）。
 *
 * 前置：调度器已启动，MX_LWIP_Init 已（或即将）由 defaultTask 执行。
 * 行为：等待 netif up（最多 3 s，期间 osDelay）→ 注册 MAC TX 完成回调（保留 CubeMX 的
 *       TxPktSemaphore 通知，并转发给 LwipPlatform 回收 TX）→ 启动 LwipPlatform 的
 *       EthLink 线程（PHY 复位与链路切换在线程内异步进行）。
 * @return 见 EthInit；仅 Ok 表示 BSP 已启动，不表示链路已建立（链路状态见 ETH_LinkUp()）。
 */
EthInit ETH_Init();

/** @brief EthInit 的稳定文本名（START/FAIL 报告用），未知值返回 "UNKNOWN"。 */
const char* eth_init_name(EthInit result);

/**
 * @brief EthLink 线程是否已因致命错误终止；可在任意线程轮询。
 *
 * 终止原因：PHY 启动失败、MDIO 读取失败（中断模式下）、MAC 启停失败等；本文件不重试，
 * 也不调用 Error_Handler，调用方据此后报 FAIL。
 */
bool ETH_PlatformFailed();

/** @brief 上一次 EthLink 致命错误码；直接对应 LwipPlatform::PollResult 的枚举值（0 = Ok）。 */
std::uint32_t ETH_PlatformError();

/** @brief ETH_PlatformError() 取值的文本名（与 PollResult 枚举一一对应）。 */
const char* eth_platform_error_name(std::uint32_t code);

/**
 * @brief 累计 MAC TX 完成回调事件数（ISR 内自增，32 位回绕，可能合并完成）。
 * @note 不是提交次数，也不承诺逐帧回调；含 ARP/ICMP 等协议栈流量，仅作整机旁证。
 *       应用送达由主机逐字节校验和 CONFIRM 证明。
 */
std::uint32_t ETH_TxCompleteCount();


/// @brief netif 链路是否 up（LwIP netif_set_link_up 的结果；读取在 core lock 内完成）。
bool ETH_LinkUp();

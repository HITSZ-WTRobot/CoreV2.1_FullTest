/**
 * @file ethernet_test.hpp
 * @brief UDP/5001 固定图案整板实测与整板报告快照读取；SWD 可作离线诊断。
 *
 * 接线：DP83822 RMII，MDIO 地址 1；PA1 REF_CLK、PA2 MDIO、PA7 CRS_DV、
 * PB11 TX_EN、PB12 TXD0、PB13 TXD1、PC1 MDC、PC4 RXD0、PC5 RXD1。
 * PB14 RESET_N；PB10 INT/PWDN_N 上拉、下降沿 EXTI15_10、NVIC priority 5。
 * GPIO/EXTI IRQ 归 CubeMX；USER_PHY 不生成 ETH MSP，eth/msp.cpp 保留 HAL-only 板级 glue。
 * 唯一 HAL_GPIO_EXTI_Callback 经 GPIO_Driver 分发给 PHY。
 * DP83822 与 LwipPlatform 保持中断模式，测试层不轮询 PHY/操作寄存器。
 * 板卡 192.168.1.48/24，主机例如 192.168.1.10/24，RJ45 直连或交换机均可。
 *
 * 主机：uv run tools/eth_test.py --board-ip 192.168.1.48 --rounds 20
 * 默认只做一次完整性测试，再实时刷新整板仪表盘；--interval 设置刷新间隔（默认 1 秒）。
 * --results-only 跳过完整性测试；--once 单次显示，非 TTY 输出也默认单次。
 * uv 自动安装脚本声明的 Rich 依赖，无需环境变量；UDP 仍为 stop-and-wait、无重传。
 * 超时/断线/字节不一致会明确显示；实时模式保留陈旧快照，下个正常采样周期重新查询。
 *
 * 线协议（所有数据报均不含隐式 NUL）：
 * - 每轮依次发送长度 8、64、256、512、1400 的 A 图案，板卡逐字节严格验证，回对应 B。
 * - 字节 0..3：A="FTA1"，B="FTB1"；字节 4：长度索引 0..4。
 *   字节 i>=5：A=(i*37 + index*19 + 0x53) mod 256，B=A xor 0xA5。
 * - 同一轮只能按上述顺序，每次收到 B 后才发下一 A；首次合法 A 绑定源 IP/端口。
 * - 最后主机发 ASCII "CONFIRM <rounds> 0\n"，rounds 为本次会话完整轮数，
 *   必须与板端已接收并提交响应的报文数严格一致。主机确认其逐字节验证过所有 B。
 *   板卡此时才累计 good 并置 PASS，回固定 "CONFIRM ACK\n"，随后允许新会话。
 * - 主机发现失败可发 "CONFIRM <completed-rounds> 1\n" 把失败写入板端；
 *   失联时无法保证这条通知到达。所有非法报文/失败确认回固定 "ERR FAIL\n"。
 * - 未收到 CONFIRM 始终不能 PASS；错误累计锁存，失败后重启板卡再验收。
 *   无鉴权，仅用于台架。固定 B 不携带序号；不用于鉴别相同用例的网络复制/重放。
 *
 * 报告协议（独立于完整性测试，ETH 已锁存 FAIL 仍可查询）：
 * - 主机发 "RESULT\n"，板端同步调用已注册 provider 生成当前文本，再冻结到 3072-byte snapshot。
 *   文本中的 uptime/计数属于该次快照时刻，不是每片接收时刻；查询不改写外设测试结果。
 * - 每片 "RESULT <session> <part> <parts> <total> <crc8>\n<body>"，所有数字十进制，
 *   part 从 0 开始，body 最多 1100 bytes；crc8 为 body 的 CRC-8/poly 0x07、init/xorout 0、无反射。
 * - 主机严格验证当前片后发 "RACK <session> <part> <crc8>\n"，板端才发送下一片；
 *   末片 ACK 后回固定 "RESULT DONE\n"。每次查询 session 递增且不复用，重启除外。
 * - 无 provider 回 "RESULT NOT_READY\n"；等待 ACK 时的新查询回 "RESULT BUSY\n"。
 *   旧/异 peer/错误 ACK 被忽略。无重发，ACK 丢失或发送失败时动态 DMA 缓冲一直冻结，
 *   只有原 peer 匹配 session/part/CRC 的迟到 ACK 可解除占用；超时不能回收。
 *   空文本也回 NOT_READY；provider 超过容量或 session 耗尽回固定 ERR FAIL，不截断快照。
 * - ACK 是可信台架主机收到本次唯一发送的证明，不是防恶意 ACK 的认证协议。
 *   ETH 完全不通时仍需 SWD 读取 g_fulltest_report；读取报告不等于完整性测试 PASS。
 *
 * DMA 契约：直接使用已有 net::UdpSocket::open/receiveFrom/sendTo，不改变包默认
 * LWIP_NETIF_TX_SINGLE_PBUF=0。所有 B/ACK/ERR/BUSY/DONE 为 MEM_DMA 中的 memory::DMABuffer，
 * AXI SRAM noncacheable；init() 显式填充一次，随后整个固件寿命内绝不改写。
 * 另有一个动态 MEM_DMA buffer，仅匹配主机 RACK 后才可复用；不能依据 MAC 完成总计数、
 * 延时或 sendTo 返回值回收。接收缓冲/snapshot 可在 DTCM。每 poll 最多接收一次、发送一次，
 * 不排队/重试；receiveFrom(...,0) 不等待，sendTo 保留原生时延语义，不承诺硬实时非阻塞。
 *
 * SWD Status：tx=整机 MAC TX 完成回调事件数（含 ARP/ICMP，可合并完成，非应用报文数），
 * 仅作发送活动旁证；真正应用送达证明是主机逐字节验证与 CONFIRM。
 * rx=收到的数据报数，good=主机确认的 B 数，bad=协议/内容/主机失败，errors=API/平台错误。
 * timeouts=0（期限由主机判定）。WAIT=无请求；RUN=会话进行/待确认；PASS=确认后无错误；
 * FAIL=启动/平台/API/协议出错。link up、socket open 或 sendTo 成功均不能单独 PASS。
 * detail：bit0 BSP成功、bit1当前link、bit2 socket、bit3平台致命错误、bit4曾link up、
 * bit5曾host确认；8..15 EthInit、16..23 PollResult、24..31 最近errno。
 * init/poll/set_report_provider 及 provider 回调仅由同一普通线程调用；init 等待 lwIP 最长约3秒。
 * 调用方每轮 osDelay(1)，让 tcpip/EthIf/EthLink 得以运行；链路处理完全由既有包负责。
 */
#pragma once

#include "test_common.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace fulltest::ethernet
{
void init();
void poll(std::uint32_t now);
// Called synchronously on RESULT in the poll owner thread; returns <=3072 bytes
// without NUL. Its storage must remain readable until this callback's caller copies it.
using ReportProvider = std::span<const char> (*)(std::uint32_t now);
void set_report_provider(ReportProvider provider) noexcept;
const Status* statuses(std::size_t& count);
} // namespace fulltest::ethernet

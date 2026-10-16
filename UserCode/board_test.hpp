/**
 * @file board_test.hpp
 * @brief GPIO_Driver LEDs/keys/PWM and I2CDriver DMA board tests.
 *
 * Call init() once after CubeMX peripheral initialization, then poll(now) from
 * the normal CMSIS thread. No GPIO mode, timer register or IRQ configuration is
 * performed here. The .ioc owns all pin pulls, DMA streams and IRQ priorities.
 *
 * Records, in order:
 * - LED_WALK: PE10/PE11/PE12 = LED0/1/2. Six one-second phases: individual
 *   LEDs blink at 4 Hz, all blink, all off, all on. tx counts complete walk
 *   cycles; detail is phase 0..5. RUN until a human observes all six phases and
 *   writes g_led_walk_confirm = kTokenLedWalk (0x4C454430) through SWD.
 * - KEY0 PB2, KEY1 PE7, KEY2 PE8, KEY3 PE9: active-low, .ioc pull-up/EXTI.
 *   Polling reads through GpioPinInput; no ISR or EXTI ownership is taken.
 *   tx/rx count debounced presses/releases; good counts paired press/releases.
 *   Two complete cycles per key are required for PASS. A key held at startup
 *   must first be released before a complete cycle can count. bad counts short
 *   bounces, detail counts debounce candidates; bounces alone cannot pass.
 * - I2C5_SCAN: PC10 SDA / PC11 SCL, externally pulled up to 3.3 V.
 *   Each poll probes at most one address (0x08..0x77, 3 ms HAL timeout; HAL tick
 *   granularity/scheduling can extend elapsed wall time). No target gives WAIT
 *   after the first scan; an address ACK also remains WAIT, not data-path PASS.
 *   EEPROM testing is disabled by default: nothing is written on an empty or
 *   unknown bus. Enable only for a confirmed 24C02 at 0x50, /WP low, whose
 *   0xF0..0xFF contents may be temporarily changed. The one-shot sequence is
 *   DMA backup -> inverted pattern page writes -> ACK polls -> DMA verification
 *   -> original-data page writes -> ACK polls -> DMA restore verification.
 *   I2CBusDMA waits for IRQ completion with a bounded RTOS thread wait; other
 *   test modules resume on the next main poll. Each poll performs at most one
 *   EEPROM transaction/probe and does not scan alongside an EEPROM operation.
 *   DMA storage is aligned DMABuffer in MEM_DMA, explicitly cleared in init().
 *   Recovery uses the package; no duplicate line-recovery or HAL init code.
 *   HAL completion/error callback forwarding is supplied by I2CBusDMA itself;
 *   the board module defines neither callback bridges nor IRQ handlers.
 *   tx counts completed DMA page writes; rx counts ACKs and completed DMA reads;
 *   good/bad count verified/mismatching bytes. errors/timeouts report failures.
 *   PASS requires BOTH pattern and restore verification. A recovered original
 *   after a failed pattern test stays FAIL. detail bit20 warns restore failure:
 *   device contents may no longer match the backup, requiring manual recovery.
 *   detail bits: 0 scan completed, 1 latest scan had ACK, 8..15 ACK count,
 *   16 EEPROM enabled, 17 EEPROM started, 18 pattern verified, 19 restore
 *   verified, 20 restore failed. g_i2c_* expose scan address masks/progress.
 * - TIM15_PWM: PE5 CH1 1 kHz / 25%, PE6 CH2 1 kHz / 75%.
 *   .ioc presets 275 MHz timer clock, PSC=4, ARR=54999, CCR1=13750, CCR2=41250.
 *   PWM_t/PWM_Start only starts each channel; its void API offers no HAL result.
 *   tx is explicitly the number of submitted start requests (2), NOT verified
 *   output. No register readback, clock derivation or CPU-cycle self-test.
 *   RUN until an oscilloscope confirms BOTH outputs and the operator writes
 *   g_tim15_confirm = kTokenTim15Pwm (0x50574D31). detail bit3 marks confirmation.
 *
 * LED/PWM have no feedback circuit: startup/configuration never causes PASS.
 * Accepted SWD tokens increment rx, become zero after consumption, and mark
 * PASS; other nonzero tokens increment bad. good stays zero for LED/PWM.
 */

#pragma once

#include "test_common.hpp"

namespace fulltest::board
{
// ---------------------------------------------------------------- 配置（可直接改）
// LED 极性：本板 LED 由引脚经限流电阻驱动，按“高电平点亮”处理。若实测相反（引脚拉低才亮），
// 置 false；走灯时序不变，只是相位含义变成“灭/亮”互换，人工判据同样成立。
inline constexpr bool kLedActiveHigh = true;

// Active-low keys; pulls and EXTI modes for all four keys are configured in .ioc.
inline constexpr bool kKeyActiveLow = true;
// 去抖窗口：电平必须连续保持 kKeyDebounceMs 才被承认为一次状态变化。窗口与 poll 周期无关
// （poll 快于窗口时要求多次连续采样，慢于窗口时下一次采样即可确认）。
inline constexpr uint32_t kKeyDebounceMs = 10;
// 判定阈值：至少观测到这么多次完整的“按下→释放”，才算按键通路可用。
inline constexpr uint32_t kKeyRequiredCycles = 2;

// I2C5 扫描范围：有效 7 位地址 0x08..0x77（0x00..0x07 与 0x78..0x7F 为保留地址）。
inline constexpr uint8_t  kScanFirstAddress = 0x08;
inline constexpr uint8_t  kScanLastAddress  = 0x77;
inline constexpr uint32_t kProbesPerPoll    = 1;   // bounded work per poll
// Per-address HAL budget; an elapsed-budget counter is diagnostic only and
// cannot distinguish bus trouble from thread preemption.
inline constexpr uint32_t kProbeTimeoutMs   = 3;
inline constexpr uint32_t kScanPeriodMsHit  = 5000;  // 上次扫到器件时的重扫周期
inline constexpr uint32_t kScanPeriodMsEmpty = 10000; // 空总线的重扫周期
// Consecutive HAL_BUSY probes before delegating recovery to I2CBusDMA.
inline constexpr uint32_t kI2cBusyBeforeRecover = 8;

// ---- 24C02（及兼容 2 线 EEPROM）可选写/读校验，默认关闭 ----
// 打开前必须确认：0x50 上确实是 EEPROM，且 kEepromTestOffset 这 kEepromTestBytes 字节
// 允许被临时改写。测试会先备份该区间，写入反码图案并回读校验，然后无条件尝试恢复原数据；
// 恢复失败会在 detail 里明确标出（bit20），此时器件内容可能与测试前不同。
// 接线：PC10=SDA / PC11=SCL 各需要一个上拉（板上已有则不加），外接 4.7 kΩ 到 3.3 V 即可；
// A0/A1/A2 全接 GND 时 7 位地址为 0x50，/WP 必须接 GND 才允许写；确认器件在 3.3 V 下可写。
// 换器件型号时按需修改 kEepromAddress / kEepromTestOffset / kEepromTestBytes / kEepromPageBytes
// （页大小不同必须同步改，跨页写会回卷覆盖本页起始字节）。
inline constexpr bool     kEepromWriteTestEnabled = false;
inline constexpr uint8_t  kEepromAddress          = 0x50; // 7 位地址，24C02 = 1010 000
inline constexpr uint8_t  kEepromTestOffset       = 0xF0; // 8 字节页对齐，避开 0x00 起始区
inline constexpr uint8_t  kEepromTestBytes        = 16;   // 0xF0..0xFF，正好 2 页
inline constexpr uint32_t kEepromPageBytes        = 8;    // 24C02 页大小，跨页写会回卷
inline constexpr uint32_t kEepromWriteCycleMs     = 20;   // 写周期 ACK 轮询上限（24C02 tWR ≤ 5 ms）
inline constexpr uint32_t kEepromOpTimeoutMs      = 5;    // DMA completion wait budget
inline constexpr uint32_t kEepromPhaseTimeoutMs   = 200;  // 单个阶段的最长时间预算
inline constexpr uint32_t kEepromTotalTimeoutMs   = 2000; // 整个 EEPROM 流程的最长时间预算

// 人工确认 token（4 字节 ASCII，便于调试器里直接填 'LED0' / 'PWM1'）。
inline constexpr uint32_t kTokenLedWalk  = 0x4C454430u; // "LED0"
inline constexpr uint32_t kTokenTim15Pwm = 0x50574D31u; // "PWM1"

// ------------------------------------------------------------------ 记录与索引
enum class Idx : std::size_t
{
    LedWalk  = 0,
    Key0     = 1,
    Key1     = 2,
    Key2     = 3,
    Key3     = 4,
    I2c5Scan = 5,
    Tim15Pwm = 6,
    Count    = 7,
};

// init() runs once after CubeMX init; poll() runs in the normal CMSIS thread.
void init();
void poll(uint32_t now);
const Status* statuses(std::size_t& count);

// ------------------------------------------------------- I2C5 扫描/EEPROM 实测快照
// 最近一次（或正在进行中的）扫描命中的 7 位地址集合：g_i2c_ack_mask[a >> 5] 的
// bit(a & 31) == 1 表示地址 a 应答了 ACK。每轮扫描开始时清零。
extern volatile uint32_t g_i2c_ack_mask[4];
extern volatile uint32_t g_i2c_last_scan_found; // 最近一次（或进行中）扫描命中地址数
extern volatile uint32_t g_i2c_scan_done;       // 0=尚未完成过完整扫描，1=已完成过
extern volatile uint32_t g_i2c_probe_index;     // 扫描进度：下一个待探测地址，>0x77 表示空闲

// ------------------------------------------------------- 人工/仪器确认注入
// 写入 kTokenLedWalk / kTokenTim15Pwm 表示“人已按上面的判据看过现象”。poll() 接受后清零。
extern volatile uint32_t g_led_walk_confirm;
extern volatile uint32_t g_tim15_confirm;
} // namespace fulltest::board

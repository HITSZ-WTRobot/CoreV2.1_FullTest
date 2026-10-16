/**
 * @file board_test.cpp
 * @brief Package-backed GPIO/PWM and I2C5 DMA board tests; see board_test.hpp.
 * Peripheral configuration and IRQ handlers belong to CubeMX-generated code.
 */
#include "board_test.hpp"

#include "I2CBusDMA.hpp"
#include "dma_buffer.hpp"
#include "gpio_driver.hpp"
#include "memory.hpp"
#include "pwm.h"

#include "i2c.h"
#include "main.h"
#include "tim.h"

namespace fulltest::board
{
// ---------------------------------------------------------------- 导出给 SWD 的快照
volatile uint32_t g_i2c_ack_mask[4]     = { 0u, 0u, 0u, 0u };
volatile uint32_t g_i2c_last_scan_found = 0u;
volatile uint32_t g_i2c_scan_done       = 0u;
volatile uint32_t g_i2c_probe_index     = 0u;
volatile uint32_t g_led_walk_confirm    = 0u;
volatile uint32_t g_tim15_confirm       = 0u;

namespace
{
/* ---------------------------------------------------------------- 记录表 */
Status records[static_cast<std::size_t>(Idx::Count)] = {
    { "LED_WALK" },
    { "KEY0" },
    { "KEY1" },
    { "KEY2" },
    { "KEY3" },
    { "I2C5_SCAN" },
    { "TIM15_PWM" },
};

bool initialised = false;

/* ---------------------------------------------------------------- 公共小工具 */
// Status 的计数字段是 volatile；C++20 起 volatile 的 ++/+= 已废弃，这里用显式读-改-写。
// 这些字段都只由本模块的 poll()（单线程）写入，不存在竞争。
void bump(volatile uint32_t& v)
{
    v = v + 1u;
}

void bump_by(volatile uint32_t& v, uint32_t n)
{
    v = v + n;
}

// Physical observations are injected through one-shot SWD tokens, never inferred.
void confirm_apply(Status& s, volatile uint32_t& field, uint32_t token)
{
    const uint32_t value = field;
    if (value == 0u)
        return;
    field = 0u;
    if (value != token)
    {
        bump(s.bad);
        return;
    }
    bump(s.rx);
    s.state = State::Pass;
}

/* =================================================================== LED 走灯 */
constexpr uint32_t kLedBlinkHalfMs = 125u; // 4 Hz
constexpr uint32_t kLedPhaseMs     = 1000u;
constexpr uint32_t kLedPhaseCount  = 6u;
constexpr uint32_t kLedCycleMs     = kLedPhaseMs * kLedPhaseCount;

uint32_t led_cycle_start = 0u;
bsp::gpio::GpioPinOutput leds[] = {
    { { LED0_GPIO_Port, LED0_Pin } },
    { { LED1_GPIO_Port, LED1_Pin } },
    { { LED2_GPIO_Port, LED2_Pin } },
};

void led_write(bool l0, bool l1, bool l2)
{
    const GPIO_PinState on  = kLedActiveHigh ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState off = kLedActiveHigh ? GPIO_PIN_RESET : GPIO_PIN_SET;
    leds[0].write(l0 ? on : off);
    leds[1].write(l1 ? on : off);
    leds[2].write(l2 ? on : off);
}

void led_poll(uint32_t now)
{
    Status& s = records[static_cast<std::size_t>(Idx::LedWalk)];

    if (elapsed(now, led_cycle_start, kLedCycleMs))
    {
        // 重新对齐周期起点（poll 卡顿后不追赶，避免相位错乱）
        led_cycle_start = now;
        bump(s.tx);
    }

    const uint32_t t     = now - led_cycle_start;
    const uint32_t phase = t / kLedPhaseMs;
    const bool     blink = ((t / kLedBlinkHalfMs) & 1u) != 0u;

    bool l0 = false;
    bool l1 = false;
    bool l2 = false;
    switch (phase)
    {
    case 0: l0 = blink; break;           // 只有 LED0 闪
    case 1: l1 = blink; break;           // 只有 LED1 闪
    case 2: l2 = blink; break;           // 只有 LED2 闪
    case 3: l0 = l1 = l2 = blink; break; // 三灯同闪
    case 4: break;                       // 三灯全灭
    default: l0 = l1 = l2 = true; break; // 三灯全亮
    }
    led_write(l0, l1, l2);

    s.detail = phase;
    confirm_apply(s, g_led_walk_confirm, kTokenLedWalk);
}

/* =================================================================== 按键去抖 */
struct KeyRun
{
    bsp::gpio::GpioPinInput gpio;
    Status*       status;
    bool          stable;        // 已确认的“按下”电平
    bool          pending;       // 存在尚未确认的相反电平
    bool          press_seen;    // 已确认按下、等待配对释放
    uint32_t      pending_since; // pending 起始时刻
};

KeyRun keys[] = {
    { { KEY0_GPIO_Port, KEY0_Pin }, &records[static_cast<std::size_t>(Idx::Key0)],
      false, false, false, 0u },
    { { KEY1_GPIO_Port, KEY1_Pin }, &records[static_cast<std::size_t>(Idx::Key1)],
      false, false, false, 0u },
    { { KEY2_GPIO_Port, KEY2_Pin }, &records[static_cast<std::size_t>(Idx::Key2)],
      false, false, false, 0u },
    { { KEY3_GPIO_Port, KEY3_Pin }, &records[static_cast<std::size_t>(Idx::Key3)],
      false, false, false, 0u },
};

bool key_level_pressed(GPIO_PinState level)
{
    return level == (kKeyActiveLow ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void key_poll(KeyRun& k, uint32_t now)
{
    Status&             s     = *k.status;
    const bool          sample = key_level_pressed(k.gpio.read());

    if (sample == k.stable)
    {
        if (k.pending)
        {
            // 反向电平没撑过去抖窗口就回到原状态 → 机械抖动/毛刺
            bump(s.bad);
            k.pending = false;
        }
        return;
    }

    if (!k.pending)
    {
        k.pending       = true;
        k.pending_since = now;
        bump(s.detail); // 原始电平跳变次数（含抖动）
        return;
    }

    if (!elapsed(now, k.pending_since, kKeyDebounceMs))
        return;

    k.stable  = sample;
    k.pending = false;
    if (k.stable)
    {
        bump(s.tx);
        k.press_seen = true;
    }
    else
    {
        bump(s.rx);
        if (k.press_seen)
        {
            k.press_seen = false;
            bump(s.good);
            if (s.good >= kKeyRequiredCycles)
                s.state = State::Pass;
        }
    }
}

/* ================================================================== I2C5 总线 */
// I2C5_SCAN 记录的 detail 位定义（board_test.hpp 里有对应表格）
constexpr uint32_t kI2cDetailScanDone      = 1u << 0;
constexpr uint32_t kI2cDetailAnyAck        = 1u << 1;
constexpr uint32_t kI2cDetailCountShift    = 8u;
constexpr uint32_t kI2cDetailEepEnabled    = 1u << 16;
constexpr uint32_t kI2cDetailEepStarted    = 1u << 17;
constexpr uint32_t kI2cDetailPatternOk     = 1u << 18;
constexpr uint32_t kI2cDetailRestoreOk     = 1u << 19;
constexpr uint32_t kI2cDetailRestoreFailed = 1u << 20;

static_assert(kScanFirstAddress <= kScanLastAddress, "I2C5 扫描地址区间错误");
static_assert(kEepromTestOffset % kEepromPageBytes == 0u, "EEPROM 测试区必须页对齐");
static_assert(kEepromTestBytes % kEepromPageBytes == 0u, "EEPROM 测试区必须整页");
static_assert(static_cast<uint32_t>(kEepromTestOffset) + kEepromTestBytes <= 256u,
              "24C02 只有 256 字节，测试区越界");

constexpr uint16_t kEepromAddr8 = static_cast<uint16_t>(kEepromAddress << 1u);
I2CBusDMA bus(&hi2c5, { GPIOC, GPIO_PIN_11, GPIOC, GPIO_PIN_10, GPIO_AF4_I2C5 });
MEM_DMA memory::DMABuffer<kEepromTestBytes> eep_backup;
MEM_DMA memory::DMABuffer<kEepromTestBytes> eep_pattern;
MEM_DMA memory::DMABuffer<kEepromTestBytes> eep_readback;

struct ScanRun
{
    uint32_t next_addr    = kScanFirstAddress; // 下一个待探测的 7 位地址
    uint32_t last_scan_ms = 0u;                // 上一轮扫描开始时刻
    uint32_t period_ms    = 0u;                // 0 → 下一轮立即开始
    uint32_t busy_streak  = 0u;
    uint32_t found        = 0u;
    bool     active       = false;
};

ScanRun scan;

bool ack_mask_get(uint8_t addr7)
{
    return (g_i2c_ack_mask[addr7 >> 5u] & (1u << (addr7 & 31u))) != 0u;
}

void ack_mask_set(uint8_t addr7)
{
    g_i2c_ack_mask[addr7 >> 5u] |= (1u << (addr7 & 31u));
}

// Recovery, including any line pulses and HAL reinitialization, is owned by the package.
void i2c_recover(Status& s)
{
    if (!bus.recover())
        bump(s.errors);
    scan.busy_streak = 0u;
}

// 探测单个 7 位地址，返回 true 表示器件对该地址给出了 ACK。
bool i2c_probe(uint8_t addr7, Status& s)
{
    const uint32_t          t0 = HAL_GetTick();
    const HAL_StatusTypeDef st =
            HAL_I2C_IsDeviceReady(&hi2c5, static_cast<uint16_t>(addr7 << 1u), 1u, kProbeTimeoutMs);
    const uint32_t dt = HAL_GetTick() - t0;

    if (st == HAL_OK)
    {
        scan.busy_streak = 0u;
        return true;
    }
    if (st == HAL_BUSY)
    {
        bump(s.errors);
        if (++scan.busy_streak >= kI2cBusyBeforeRecover)
            i2c_recover(s);
        return false;
    }

    // A missing address is expected, not a data-path failure. Budget exhaustion
    // is diagnostic only: preemption can also increase observed elapsed time.
    if (dt >= kProbeTimeoutMs)
        bump(s.timeouts);
    scan.busy_streak = 0u;
    return false;
}

/* ------------------------------------------------- 可选 24C02（EEPROM）读写校验 */
enum class EepPhase : uint32_t
{
    Off = 0,
    WaitDevice,
    Backup,
    WritePage,
    AckPoll,
    RestoreReady,
    VerifyWrite,
    VerifyRestore,
    Terminal, // 终态：不再碰总线
};

struct EepState
{
    EepPhase phase{ kEepromWriteTestEnabled ? EepPhase::WaitDevice : EepPhase::Off };
    uint32_t page{ 0u };
    uint32_t ack_start{ 0u };
    uint32_t phase_start{ 0u };
    uint32_t total_start{ 0u };
    bool     restoring{ false };   // 当前写的是备份数据（恢复流程）
    bool     writes_done{ false }; // 已经改动过器件内容
    bool     test_failed{ false }; // A successful restore must not hide a failed pattern test.
};

EepState eep;

uint8_t eep_offset(uint32_t page)
{
    return static_cast<uint8_t>(kEepromTestOffset + page * kEepromPageBytes);
}

// 失败处理：置 FAIL 并决定下一步。器件内容已被改动时无条件转入恢复流程；
// 恢复流程自己失败则明确标记“恢复失败”（内容可能与测试前不同）。
void eep_fail(Status& s, uint32_t now)
{
    eep.test_failed = true;
    s.state = State::Fail;
    if (eep.restoring)
        s.detail |= kI2cDetailRestoreFailed;
    if (eep.writes_done && !eep.restoring)
    {
        // 器件内容已被改动：无论如何都要尝试恢复，并给它一份独立的时间预算
        eep.restoring   = true;
        eep.page        = 0u;
        // A failed/partial page may still be in its internal write cycle.
        eep.phase       = EepPhase::RestoreReady;
        eep.ack_start   = now;
        eep.phase_start = now;
        eep.total_start = now;
        return;
    }
    eep.phase = EepPhase::Terminal;
}

bool eep_read_region(uint8_t offset, uint8_t* dst, Status& s)
{
    if (!bus.memRead(kEepromAddress, offset, dst, kEepromTestBytes, kEepromOpTimeoutMs))
    {
        bump(s.errors);
        if (bus.lastError() == I2CBusDMA::Error::Timeout)
            bump(s.timeouts);
        return false;
    }
    bump(s.rx);
    return true;
}

uint32_t eep_mismatch(const uint8_t* a, const uint8_t* b)
{
    uint32_t mismatch = 0u;
    for (uint32_t i = 0u; i < kEepromTestBytes; ++i)
    {
        if (a[i] != b[i])
            ++mismatch;
    }
    return mismatch;
}

void eeprom_poll(uint32_t now, Status& s, bool& bus_used)
{
    if (eep.phase == EepPhase::Off || eep.phase == EepPhase::Terminal)
        return;
    bus_used = eep.phase != EepPhase::WaitDevice;

    if (eep.phase != EepPhase::WaitDevice &&
        (elapsed(now, eep.phase_start, kEepromPhaseTimeoutMs) ||
         elapsed(now, eep.total_start, kEepromTotalTimeoutMs)))
    {
        bump(s.timeouts);
        eep_fail(s, now);
        return;
    }

    switch (eep.phase)
    {
    case EepPhase::WaitDevice:
        // 器件不在总线上：完全不碰总线，继续等（扫描会给出 ACK 信息）
        if (!ack_mask_get(kEepromAddress))
            return;
        eep.phase       = EepPhase::Backup;
        eep.total_start = now;
        eep.phase_start = now;
        s.detail |= kI2cDetailEepStarted;
        return;

    case EepPhase::Backup:
        bus_used = true;
        if (!eep_read_region(kEepromTestOffset, eep_backup.data(), s))
        {
            eep_fail(s, now); // 还没写过任何东西，无需恢复
            return;
        }
        for (uint32_t i = 0u; i < kEepromTestBytes; ++i)
            eep_pattern.data()[i] = static_cast<uint8_t>(~eep_backup.data()[i]);
        eep.restoring   = false;
        eep.page        = 0u;
        eep.phase       = EepPhase::WritePage;
        eep.phase_start = now;
        return;

    case EepPhase::WritePage:
    {
        bus_used           = true;
        const uint8_t* src = eep.restoring ? eep_backup.data() : eep_pattern.data();
        if (!eep.restoring)
            eep.writes_done = true; // 写请求一旦发出，器件内容就可能已改变
        if (!bus.memWrite(kEepromAddress, eep_offset(eep.page),
                          src + eep.page * kEepromPageBytes, kEepromPageBytes, kEepromOpTimeoutMs))
        {
            bump(s.errors);
            if (bus.lastError() == I2CBusDMA::Error::Timeout)
                bump(s.timeouts);
            eep_fail(s, now);
            return;
        }
        bump(s.tx); // 已完成的页写
        eep.ack_start = now;
        eep.phase     = EepPhase::AckPoll;
        return;
    }

    case EepPhase::RestoreReady:
        if (HAL_I2C_IsDeviceReady(&hi2c5, kEepromAddr8, 1u, kProbeTimeoutMs) == HAL_OK)
        {
            bump(s.rx);
            eep.phase = EepPhase::WritePage;
            eep.phase_start = now;
        }
        else if (elapsed(now, eep.ack_start, kEepromWriteCycleMs))
        {
            bump(s.timeouts);
            eep_fail(s, now);
        }
        return;

    case EepPhase::AckPoll:
    {
        bus_used = true;
        // 24C02 在内部写周期（tWR ≤ 5 ms）内不应答任何地址，ACK 出现即写入完成
        if (HAL_I2C_IsDeviceReady(&hi2c5, kEepromAddr8, 1u, kProbeTimeoutMs) == HAL_OK)
        {
            bump(s.rx);
            ++eep.page;
            if (eep.page < kEepromTestBytes / kEepromPageBytes)
                eep.phase = EepPhase::WritePage;
            else
                eep.phase = eep.restoring ? EepPhase::VerifyRestore : EepPhase::VerifyWrite;
            eep.phase_start = now;
            return;
        }
        if (elapsed(now, eep.ack_start, kEepromWriteCycleMs))
        {
            bump(s.timeouts); // 写周期内器件始终不应答
            eep_fail(s, now);
        }
        return;
    }

    case EepPhase::VerifyWrite:
        bus_used = true;
        if (!eep_read_region(kEepromTestOffset, eep_readback.data(), s))
        {
            eep_fail(s, now); // 已写过 → eep_fail 会转入恢复
            return;
        }
        if (const uint32_t mismatch = eep_mismatch(eep_readback.data(), eep_pattern.data()); mismatch != 0u)
        {
            bump_by(s.bad, mismatch);
            eep_fail(s, now);
            return;
        }
        bump_by(s.good, kEepromTestBytes);
        s.detail |= kI2cDetailPatternOk;
        eep.restoring   = true; // 图案校验通过，开始恢复原数据
        eep.page        = 0u;
        eep.phase       = EepPhase::WritePage;
        eep.phase_start = now;
        return;

    case EepPhase::VerifyRestore:
        bus_used = true;
        if (!eep_read_region(kEepromTestOffset, eep_readback.data(), s))
        {
            eep_fail(s, now);
            return;
        }
        if (const uint32_t mismatch = eep_mismatch(eep_readback.data(), eep_backup.data()); mismatch != 0u)
        {
            bump_by(s.bad, mismatch);
            eep_fail(s, now); // restoring → 标记“恢复失败”
            return;
        }
        bump_by(s.good, kEepromTestBytes);
        s.detail |= kI2cDetailRestoreOk;
        s.state = eep.test_failed ? State::Fail : State::Pass;
        eep.phase = EepPhase::Terminal;
        return;

    case EepPhase::Off:
    case EepPhase::Terminal:
        return;
    }
}

/* --------------------------------------------------------- 地址扫描 / 状态汇总 */
void scan_poll(uint32_t now, Status& s, bool bus_used)
{
    if (bus_used)
        return; // 本 poll 的总线已被 EEPROM 流程占用，下一轮再扫

    if (!scan.active)
    {
        if (!elapsed(now, scan.last_scan_ms, scan.period_ms))
            return;
        scan.active       = true;
        scan.next_addr    = kScanFirstAddress;
        scan.found        = 0u;
        scan.last_scan_ms = now;
        for (uint32_t i = 0u; i < 4u; ++i)
            g_i2c_ack_mask[i] = 0u; // 本次扫描结果不叠加到上一次
        g_i2c_last_scan_found = 0u;
        g_i2c_probe_index     = kScanFirstAddress;
    }

    for (uint32_t i = 0u; i < kProbesPerPoll; ++i)
    {
        if (scan.next_addr > kScanLastAddress)
        {
            g_i2c_scan_done       = 1u;
            g_i2c_last_scan_found = scan.found;
            g_i2c_probe_index     = kScanLastAddress + 1u;
            s.detail |= kI2cDetailScanDone;
            s.detail = (s.detail & ~kI2cDetailAnyAck) | (scan.found != 0u ? kI2cDetailAnyAck : 0u);
            s.detail &= ~(0xFFu << kI2cDetailCountShift);
            s.detail |= (scan.found & 0xFFu) << kI2cDetailCountShift;
            scan.active    = false;
            scan.period_ms = (scan.found != 0u) ? kScanPeriodMsHit : kScanPeriodMsEmpty;
            return;
        }

        if (i2c_probe(static_cast<uint8_t>(scan.next_addr), s))
        {
            ack_mask_set(static_cast<uint8_t>(scan.next_addr));
            ++scan.found;
            g_i2c_last_scan_found = scan.found;
            bump(s.rx);
        }
        ++scan.next_addr;
        g_i2c_probe_index = scan.next_addr;
    }
}

void i2c_poll(uint32_t now)
{
    Status& s = records[static_cast<std::size_t>(Idx::I2c5Scan)];

    bool bus_used = false;
    eeprom_poll(now, s, bus_used);
    scan_poll(now, s, bus_used);

    if (eep.phase == EepPhase::Terminal)
        return; // EEPROM 流程已给出终态（PASS/FAIL），不再被扫描结果覆盖
    if (eep.phase != EepPhase::Off && eep.phase != EepPhase::WaitDevice)
    {
        s.state = eep.test_failed ? State::Fail : State::Running;
        return;
    }
    // 首次扫描尚未跑完 → 测试进行中
    if (g_i2c_scan_done == 0u)
    {
        s.state = State::Running;
        return;
    }
    // 扫描完成：无论是否扫到地址都停在 WAIT —— 地址 ACK 不等于数据通路可用
    s.state = State::Waiting;
}

/* ================================================================== TIM15 PWM */
PWM_t pwm_outputs[] = { { &htim15, TIM_CHANNEL_1 }, { &htim15, TIM_CHANNEL_2 } };

void tim15_init()
{
    Status& s = records[static_cast<std::size_t>(Idx::Tim15Pwm)];
    // PWM_Start returns void: count submitted start requests, not verified outputs.
    // PSC/ARR/CCR1/CCR2 are entirely owned by the .ioc configuration.
    for (PWM_t& output : pwm_outputs)
    {
        PWM_Start(&output);
        bump(s.tx);
    }
    s.state = State::Running;
}

void tim15_poll()
{
    Status& s = records[static_cast<std::size_t>(Idx::Tim15Pwm)];
    confirm_apply(s, g_tim15_confirm, kTokenTim15Pwm);
    if (s.state == State::Pass)
        s.detail = s.detail | (1u << 3);
}

} // namespace

/* ==================================================================== 对外接口 */
void init()
{
    const uint32_t now = HAL_GetTick();

    /* LED：走灯从全灭开始 */
    led_cycle_start = now;
    led_write(false, false, false);
    records[static_cast<std::size_t>(Idx::LedWalk)].state = State::Running;

    /* KEY：上拉/EXTI 模式由 .ioc 配置，只读取电平。 */
    for (KeyRun& k : keys)
    {
        k.stable        = key_level_pressed(k.gpio.read());
        k.pending       = false;
        k.press_seen    = false;
        k.pending_since = now;
        k.status->state = State::Running;
    }

    /* I2C5：扫描立刻开始，EEPROM 流程等器件出现在总线上 */
    for (std::size_t i = 0u; i < kEepromTestBytes; ++i)
    {
        eep_backup.data()[i] = 0u;
        eep_pattern.data()[i] = 0u;
        eep_readback.data()[i] = 0u;
    }
    scan.active       = false;
    scan.last_scan_ms = now;
    scan.period_ms    = 0u;
    scan.found        = 0u;
    scan.busy_streak  = 0u;
    g_i2c_probe_index = kScanLastAddress + 1u;
    if (kEepromWriteTestEnabled)
        records[static_cast<std::size_t>(Idx::I2c5Scan)].detail |= kI2cDetailEepEnabled;
    records[static_cast<std::size_t>(Idx::I2c5Scan)].state = State::Running;

    /* TIM15：仅启动 .ioc 预设的两个 PWM 通道，无软件配置/速率自检。 */
    tim15_init();

    initialised = true;
}

void poll(uint32_t now)
{
    if (!initialised)
        return; // init() 未调用：保持 START，绝不误报

    led_poll(now);
    for (KeyRun& key : keys)
        key_poll(key, now);
    tim15_poll();
    i2c_poll(now);
}

const Status* statuses(std::size_t& count)
{
    count = static_cast<std::size_t>(Idx::Count);
    return records;
}
} // namespace fulltest::board

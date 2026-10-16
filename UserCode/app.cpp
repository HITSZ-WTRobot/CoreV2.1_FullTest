#include "board_test.hpp"
#include "can_test.hpp"
#include "ethernet_test.hpp"
#include "uart_test.hpp"

#include "cmsis_os2.h"
#include "main.h"
#include "tim.h"
#include "watchdog.hpp"

#include <cstdarg>
#include <cstdio>
#include <span>

/*
 * Full-board bring-up firmware (no motor control).
 *
 * Quick connections, with the board powered off while making connections:
 * - CAN1/2/3: join all CAN_H, join all CAN_L, common ground, 120 ohm at
 *   each physical end (about 60 ohm measured H-to-L with power removed).
 * - TTL UARTs are crossed between DIFFERENT peripherals, never self-looped:
 *   UART9 PD15(TX) -> USART3 PD9(RX); USART3 PD8(TX) -> UART9 PD14(RX).
 *   USART6 PC6(TX) -> USART10 PE2(RX); USART10 PE3(TX) -> USART6 PC7(RX).
 *   USART1 PA9(TX) -> UART4 PA11(RX); UART4 PA12(TX) -> USART1 PA10(RX).
 *   Each pair must pass both directions. Common ground, 3.3 V TTL only;
 *   never connect an RS232 voltage level or tie two TX outputs together.
 * - USART2 is RS485: use its A/B connector and tools/rs485_echo.py.
 *   Do not short A/B. Keep the board's hardware-DE configuration.
 * - Ethernet: PC on 192.168.1.x/24, board 192.168.1.48; run
 *   uv run tools/eth_test.py for integrity verification once, then live results;
 *   add --results-only to skip that check, or --once for a single dashboard.
 * - Press AND release each of KEY0..KEY3 twice; observe the three LEDs.
 * - I2C5 defaults to non-destructive address scanning. No external target
 *   means WAIT, not PASS; board_test.hpp describes the optional EEPROM test.
 * - TIM15: scope PE5/PE6 to ground; board_test.hpp gives the expected PWM.
 *   LED/PWM stay RUN until observed: optional SWD confirmation tokens are
 *   fulltest::board::g_led_walk_confirm=0x4C454430 and
 *   fulltest::board::g_tim15_confirm=0x50574D31, after visual/scope checks.
 *
 * A RESULT request obtains a fresh frozen report through the existing UDP socket.
 * All module status tables and g_fulltest_report remain readable via SWD too.
 * RUN on LEDs/PWM means output is driven, NOT that a physical measurement
 * passed. PASS is never inferred from configuration or a missing cable.
 * Results are cumulative; disconnect/recovery is reflected by current state.
 * Historical bad/error/timeout counts remain visible after recovery.
 */

struct FulltestStatusGroup
{
    const char* name;
    const fulltest::Status* records;
    std::size_t count;
};

// Stable, named debugger entry points; no printf retargeting or console needed.
FulltestStatusGroup g_fulltest_groups[4]{};
char g_fulltest_report[3072]{};
std::size_t g_fulltest_report_length{0};
volatile uint32_t g_fulltest_uptime_ms{0};
volatile uint32_t g_fulltest_watchdog_error{HAL_OK};

namespace
{
std::size_t report_offset{0};
bool report_truncated{false};

// UartRxSync uses the existing package's millisecond Watchdog service.
// TIM5 frequency, IRQ and callback support are generated from the IOC.
void watchdog_tick(TIM_HandleTypeDef*)
{
    service::Watchdog::EatAll();
}

void append_report(const char* format, ...)
{
    if (report_truncated)
        return;
    const std::size_t available = sizeof(g_fulltest_report) - report_offset;
    va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(g_fulltest_report + report_offset, available, format, args);
    va_end(args);
    if (written < 0 || static_cast<std::size_t>(written) >= available)
    {
        report_truncated = true;
        return;
    }
    report_offset += static_cast<std::size_t>(written);
}

void make_report(uint32_t now)
{
    report_offset = 0;
    report_truncated = false;
    append_report("FULLTEST uptime_ms=%lu watchdog_hal=%lu (live snapshots; cumulative counters)\n",
                  static_cast<unsigned long>(now),
                  static_cast<unsigned long>(g_fulltest_watchdog_error));
    for (const auto& group : g_fulltest_groups)
    {
        for (std::size_t i = 0; i < group.count; ++i)
        {
            const auto& s = group.records[i];
            append_report("%s %s tx=%lu rx=%lu ok=%lu bad=%lu err=%lu tmo=%lu detail=0x%08lX\n",
                          s.name, fulltest::state_name(s.state),
                          static_cast<unsigned long>(s.tx), static_cast<unsigned long>(s.rx),
                          static_cast<unsigned long>(s.good), static_cast<unsigned long>(s.bad),
                          static_cast<unsigned long>(s.errors), static_cast<unsigned long>(s.timeouts),
                          static_cast<unsigned long>(s.detail));
        }
    }
    if (report_truncated)
    {
        constexpr char marker[] = "\nREPORT TRUNCATED\n";
        constexpr std::size_t end = sizeof(g_fulltest_report) - sizeof(marker);
        for (std::size_t i = 0; i < sizeof(marker); ++i)
            g_fulltest_report[end + i] = marker[i];
        report_offset = sizeof(g_fulltest_report) - 1;
    }
    g_fulltest_report_length = report_offset;
}

// Called synchronously by the Ethernet owner task, never from an ISR.
// The module copies this stable storage before poll() returns; later periodic
// report refreshes cannot change a report already being transmitted.
std::span<const char> report_snapshot(uint32_t now)
{
    make_report(now);
    return {g_fulltest_report, g_fulltest_report_length};
}
}

extern "C" [[noreturn]] void Init(void* argument)
{
    (void)argument;

    fulltest::uart::init();
    fulltest::board::init();
    fulltest::can::init();
    fulltest::ethernet::init();

    HAL_StatusTypeDef watchdog_status = HAL_TIM_RegisterCallback(
        &htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, watchdog_tick);
    if (watchdog_status == HAL_OK)
        watchdog_status = HAL_TIM_Base_Start_IT(&htim5);
    g_fulltest_watchdog_error = static_cast<uint32_t>(watchdog_status);

    g_fulltest_groups[0].name = "ETH";
    g_fulltest_groups[0].records = fulltest::ethernet::statuses(g_fulltest_groups[0].count);
    g_fulltest_groups[1].name = "CAN";
    g_fulltest_groups[1].records = fulltest::can::statuses(g_fulltest_groups[1].count);
    g_fulltest_groups[2].name = "UART";
    g_fulltest_groups[2].records = fulltest::uart::statuses(g_fulltest_groups[2].count);
    g_fulltest_groups[3].name = "BOARD";
    g_fulltest_groups[3].records = fulltest::board::statuses(g_fulltest_groups[3].count);

    uint32_t last_report = HAL_GetTick();
    make_report(last_report);
    fulltest::ethernet::set_report_provider(report_snapshot);
    for (;;)
    {
        const uint32_t now = HAL_GetTick();
        g_fulltest_uptime_ms = now;
        fulltest::uart::poll(now);
        fulltest::can::poll(now);
        fulltest::board::poll(now);
        fulltest::ethernet::poll(now);
        if (fulltest::elapsed(now, last_report, 1000))
        {
            make_report(now);
            last_report = now;
        }
        osDelay(1);
    }
}

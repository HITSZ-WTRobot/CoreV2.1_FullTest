#pragma once

#include "test_common.hpp"

#include <cstddef>
#include <cstdint>

/*
 * fulltest::uart — three bidirectional cross-UART pairs and one RS485 host test.
 *
 * 115200 8N1, no flow control. CubeMX owns peripheral initialization, circular
 * RX DMA, UART/DMA IRQ priority 5, UART callback registration enablement and
 * USART2 hardware DE timing (16 sample units before/after the frame). This
 * module only registers HAL RX/error/TX-complete callbacks and starts transfers;
 * it never defines vectors, configures NVIC, reinitializes a UART or touches
 * registers. Call init() after all generated MX_*_Init() functions.
 *
 * Receiver: protocol::UartRxSync<1,15,true>, header '$'. The package performs
 * initial byte-IT header search, DMA reception and resynchronization/error
 * recovery; the test layer only validates the complete frame and verdict.
 * Seven memory::DMABuffer<15> objects live in MEM_DMA (.RAM_AXI_Section), within
 * the board's non-cacheable AXI MPU region. Startup does not copy this custom
 * section, so init() explicitly clears its storage before starting RX. DMABuffer
 * alignment alone does not provide cache coherency; keep the generated MPU/linker
 * contract when changing the board.
 * TX uses HAL_UART_Transmit_IT and retains its ordinary-RAM buffer until TC.
 *
 * Each derived receiver has a libs::RingBuffer<Frame,5,false> SPSC queue (four
 * usable slots). RX/DMA callbacks enqueue the complete 15-byte frame, validated
 * fields and HAL_GetTick() arrival timestamp. Equal-priority UART/DMA IRQs must
 * not nest on one receiver. Malformed frames, HAL errors or queue overflow
 * invalidate a pending exchange and break the consecutive-pass streak.
 *
 * The package watchdog timeout is explicitly 500 milliseconds. The application
 * must invoke service::Watchdog::EatAll() once per real millisecond (TIM5 periodic
 * callback); do not also decrement it in poll(). isConnected() is diagnostic
 * only and is never sufficient for PASS.
 *
 * WIRING — six separate 3.3 V TTL wires, common ground; NEVER local TX-to-RX:
 *   UART9 PD15 TX -> USART3 PD9 RX; USART3 PD8 TX -> UART9 PD14 RX
 *   USART6 PC6 TX -> USART10 PE2 RX; USART10 PE3 TX -> USART6 PC7 RX
 *   USART1 PA9 TX -> UART4 PA11 RX; UART4 PA12 TX -> USART1 PA10 RX
 * Both wires in each pair are required. Never connect two TX outputs together.
 * Record indices: USART1=0, USART2=1, USART3=2, UART4=3, USART6=4,
 * UART9=5, USART10=6. TTL peers are 0<->3, 5<->2 and 4<->6.
 * USART2 (PD5 TX, PD6 RX, PD4 hardware DE) is not a TTL pair: connect its
 * external RS485 A/B to a real USB-RS485 peer, common ground, and 120-ohm
 * termination at both ends. Run: python3 tools/rs485_echo.py --port /dev/ttyUSB0
 * Do not short USART2 logic TX/RX or A/B instead of using the external host.
 *
 * FIXED ASCII PROTOCOL — unchanged host-tool contract (15 bytes)
 *   '$' src(2 hex) kind(1) seq(4 hex) pattern(4 hex) crc(2 hex) '\n'
 * src 00..06 identifies a board port; FF identifies the external host. The board
 * emits kind R; the host answers kind A, preserving sequence and pattern while
 * recomputing CRC. CRC-8 is crc::CRCX<8,0x07,0x00,false,false,0x00>::calc over
 * bytes [1..11]. Sequence and its derived pattern change on every request.
 * Each TTL endpoint emits its own R frame, without echoing or answering RX.
 * A receiver must match its configured peer's real pending sequence/pattern,
 * not its own request or merely a matching source byte. USART2 requires FF/A
 * feedback; its own R echo is diagnostic only and can never produce PASS.
 * No '#' diagnostic lines are transmitted: inserting variable-length text into
 * the fixed DMA stream would force an avoidable package resynchronization.
 *
 * TIMING / VERDICTS
 * Requests run every ~120 ms, initial delay 60 ms. Pair phases: USART1/UART4
 * 0 ms, UART9/USART3 20 ms, USART6/USART10 40 ms; USART2 phase 60 ms. Both
 * TTL endpoints start each round together, with independent source/sequence/TX
 * completion. Deadline is 60 ms from the source's round start. The next round
 * also waits at least 6 ms since both receivers' last complete frames. This is
 * a frame-based guard, not a byte-level idle measurement; package code owns
 * partial/noisy traffic recovery. ISR arrival timestamps and source TX TC ticks
 * decide deadlines even if polling runs late. An early matching RX stays in its
 * original queue slot until source TC or timeout; RX alone cannot earn credit.
 * Each TTL pair needs three consecutive rounds with BOTH directions verified
 * and both source TXs completed on time before EITHER record reports PASS.
 * A mismatch/error/timeout breaks the pair's streak; each direction still has
 * independent local counters. Reconnection can earn three clean rounds again.
 * Five local failures in either direction yield WAIT before the pair's first
 * successful round, or FAIL afterward. USART2 uses the same three-round rule
 * with the external host. Missing initialization/DMA/callback setup is FAIL.
 *
 * STATUS
 * tx       actual TX TC completions, not submissions
 * rx       well-formed frames admitted to the event queue
 * good     this receiver verified its peer's timely R and actual source TX TC;
 *          USART2 verified FF/A plus its own TX TC. Not a group-success count.
 * bad      local malformed frames + wrong/late/duplicate/unsolicited valid frames
 * errors   local callback/start/TX API failures + missing/late local TX TC rounds
 *          + HAL error callbacks + queue overflow (not a peer's TX errors)
 * timeouts local pending RX round ended without verified feedback; includes
 *          received data that could not be credited because source TC was absent
 * detail   bit0 USART2 local R echo seen; bit1 this RX direction ever verified;
 *          bit2 USART2 FF/A host seen; bit3 local queue overflow ever seen;
 *          bit4 package receiver currently connected; bits5..7 reserved zero;
 *          bits8..15 last local HAL error; bits16..31 saturated local TX-failure
 *          count (TX API failure or missing/late TC, counted once per round).
 * Paired records share the group verdict, not counters. For example, an open
 * TX->peer-RX wire can leave the source tx advancing but peer good stalled and
 * peer timeouts growing, with zero source TX failures if its TC IRQ still fires.
 * TTL crossing verifies the actual two-peripheral path, not voltage margins.
 * RS485 success proves a host round trip, not EMC/cable-length limits.
 * Baud/parity/flow-control variants are not tested.
 */
namespace fulltest::uart
{
inline constexpr std::size_t port_count = 7;
inline constexpr std::size_t frame_length = 15;

// Register callbacks and start package receivers after generated hardware init.
void init();

// Advance cross-UART pairs and the RS485 host exchange without blocking.
void poll(uint32_t now);

// Permanent SWD-visible records, in the port-index order documented above.
const Status* statuses(std::size_t& count);
} // namespace fulltest::uart

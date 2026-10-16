/**
 * @file    can_test.hpp
 * @author  syhanjin
 * @date    2026-10-15
 * @brief   CAN1 / CAN2 / CAN3 cross-controller bring-up test (no motor protocol).
 *
 * Wiring (power the board off before touching the bus)
 * ---------------------------------------------------
 *   CAN1  MCU PB8/PB9    CAN2  MCU PB5/PB6    CAN3  MCU PD12/PD13
 *   Each controller leaves the board through its own transceiver. Join the
 *   three transceivers into ONE physical bus:
 *     - all three CAN_H pins (transceiver side) on one wire,
 *     - all three CAN_L pins on the other wire,
 *     - a solid common ground between all three transceivers (they already
 *       share the PCB ground here; keep the ground wire next to the CAN pair
 *       when the loop leaves the board),
 *     - 120 ohm termination at BOTH physical ends of the loop, two
 *       terminators in total. With power removed, CAN_H to CAN_L reads about
 *       60 ohm; 120 ohm means one terminator is missing, 40 ohm means one too
 *       many. Missing/extra termination shows up here only as errors and
 *       timeouts, not as a dedicated measurement.
 *   The controllers stay in HAL normal mode with the transceivers active; no
 *   internal loopback is used anywhere, so a dead transceiver, swapped H/L or
 *   an open wire fails the test instead of passing silently. Do not connect a
 *   PC CAN tool unless it is configured for the same bit rates and does not
 *   drive the bus during the test.
 *
 * Clock and bit rates (owned by the CubeMX .ioc)
 * ---------------------------------------------
 *   HSE 8 MHz -> PLL1 (M = 1, N = 68, FRACN = 6144 of 8192) -> VCO 550 MHz.
 *   pll1_q_ck = VCO / 5 = 110 MHz is the FDCAN kernel clock
 *   (RCC_FDCANCLKSOURCE_PLL in HAL_FDCAN_MspInit), and all three controllers
 *   use that same 110 MHz, so their bit timing is identical by construction.
 *     Nominal: prescaler 2, 1 + 43 + 11 = 55 tq of 18.18 ns
 *              => exactly 1.000 Mbit/s, sample point 44/55 = 80.0 %.
 *     Data:    prescaler 2, BRS, 1 + 5 + 5 = 11 tq
 *              => exactly 5.000 Mbit/s, sample point 6/11 = 54.5 %.
 *   Both rates, message RAM offsets 0/512/1024 words, four standard filters,
 *   automatic retransmission, eight 64-byte RX FIFO0 and TX FIFO elements,
 *   and IT0/IT1 priority 5 are generated from the .ioc. This module does not
 *   reinitialize HAL handles, modify Init fields, configure NVIC or define IRQs.
 *     TDC (SSP) is enabled with TDCO = 12 mtq, filter window 0 mtq: at 200 ns
 *   per data bit the transceiver loop delay must be compensated for the
 *   transmitter's own bit monitoring. 12 mtq at 110 MHz is about 109 ns of
 *   extra offset after the delay the hardware measures itself. If an external
 *   tool reports form/stuff errors in the data phase while the nominal phase
 *   is clean, re-check TDCO on a scope before blaming the wiring.
 *
 * Frame protocol
 * --------------
 *   ID 0x300 + i : classic CAN 2.0 data frame, 8-byte payload, i = 0..2.
 *   ID 0x310 + i : CAN FD data frame with BRS, 64-byte payload, i = 0..2.
 *   payload[0]       = 0xA5 magic
 *   payload[1]       = source index + 1 (1..3)
 *   payload[2]       = class (0 = classic, 1 = FD)
 *   payload[3]       = sequence, one counter per source per class, mod 256
 *   payload[4]       = payload length (8 or 64)
 *   payload[5..n-2]  = pattern byte for position k:
 *                      (src*0x37 ^ class*0x5B ^ seq*0x11 ^ k*0x1D ^ 0xA5) & 0xFF
 *   payload[n-1]     = CRC-8, polynomial 0x07, initial value 0x00, no reflection
 *                      or final XOR, over payload[0..n-2]; calculated with
 *                      libs::CRC's crc::CRCX<8,0x07,0x00,false,false,0x00>.
 *   src and seq are an 8-bit counter pair, so a receiver can prove it saw
 *   *that* frame, not merely some frame: content, position and order are all
 *   checked. Each controller sends its classic frame every 20 ms (phase 0/3/6
 *   ms) and its FD frame every 40 ms (phase 0/5/10 ms); the phases keep the
 *   three transmitters out of each other's way, and the CAN driver retransmits
 *   automatically on arbitration loss or a bus error.
 *
 * How each controller is judged (three independent records)
 * ---------------------------------------------------------
 *   The four ID filters accept exactly two IDs per peer, so a controller never
 *   counts its own frames, and non-participating traffic is rejected in
 *   hardware (GlobalFilter rejects every non-matching standard/extended ID and
 *   both remote frame types).
 *   A controller reaches PASS only when all of the following hold at the same
 *   time (evaluated live, every poll):
 *     - at least 8 of its own classic AND 8 of its own FD frames left the TX
 *       FIFO (tx_classic / tx_fd: transmission completed on an acknowledged
 *       bus, not merely queued), and
 *     - each of the other two controllers delivered at least 8 verified
 *       classic frames AND at least 8 verified FD frames to it
 *       (verified_classic[] / verified_fd[]), and
 *     - no mismatch, bus-off, stuck-TX window or starvation timeout happened
 *       in the last 2 s (a link that recovered must run clean before PASS),
 *       and no bus-off has been latched since reset.
 *   One frame from one peer can therefore never produce PASS, and PASS always
 *   requires real feedback from *both* peers in *both* classes through the
 *   external transceivers.
 *   FAIL is reported when a mismatch, an error, a timeout or a bus-off was
 *   recorded in the last 2 s, or when the 8 s deadline expired while the peers
 *   were being heard (so this controller really is the broken one). The state
 *   follows the counters, so a link that recovers without bus-off runs clean
 *   for 2 s and then reaches PASS again. Bus-off requires a board reset.
 *   WAIT is reported when the deadline expired and *no* controller on the bus
 *   ever verified a frame from another controller: the whole bus stayed silent,
 *   which is what missing wiring, missing termination or dead transceivers look
 *   like from the MCU. WAIT is never a pass, and the bus-off/timeout counters
 *   remain visible in the same record so a silent bus caused by a real fault is
 *   still obvious.
 *
 * Record fields (see test_common.hpp for the shared Status layout)
 * ---------------------------------------------------------------
 *   name = "CAN1" / "CAN2" / "CAN3"
 *   state: START before init, RUN while the test is in progress, PASS when the
 *          conditions above are met, WAIT when the bus stayed silent, FAIL when
 *          this controller failed or the deadline expired with bus traffic.
 *   tx    = frames whose TX FIFO slot was released (classic + FD), i.e. TX
 *           completed; details in Counters::tx_classic / tx_fd.
 *   rx    = frames delivered to this controller after ID filtering.
 *   good  = frames that passed every content, pattern, CRC and sequence check.
 *   bad   = content / sequence mismatches (a lost frame is a sequence
 *           mismatch, so a frame repeated or dropped by the wiring counts).
 *   errors= driver setup/send failures and latched bus-off observations;
 *           send() back-pressure is separately visible as Counters::submit_busy.
 *   timeouts = 200 ms starvation windows in which a peer produced no verified
 *           frame of a class, or in which frames were pending in the TX FIFO
 *           without a single slot being released (a frame that cannot be
 *           acknowledged keeps the FIFO head busy); counted at most once per
 *           window and folded into the same field so a stuck transmitter cannot
 *           look like an idle bus.
 *   detail = bit field, defined in can_test.cpp:
 *           bit0 classic TX minimum reached, bit1 FD TX minimum reached,
 *           bit2 classic verified from both peers, bit3 FD verified from both
 *           peers, bits[7:4] how many of the two peers delivered enough
 *           verified classic frames (0..2), bits[11:8] same for FD,
 *           bits[15:12] init failure reason code (0 = none),
 *           bit16 no controller ever verified any peer frame (silent bus).
 *
 * Limitations
 * -----------
 *   - No recovery path in this module: the CAN driver exposes no stop/restart
 *     API and no abort/cancel API, so a frame that cannot be acknowledged keeps
 *     the TX FIFO head busy, the frames behind it are not transmitted (hardware
 *     FIFO order), and a controller that went bus-off is left as it is. That is
 *     reported, never hidden: send() back-pressure is counted in
 *     Counters::submit_busy, the bus-off in Counters::busoff, the 200 ms
 *     windows without any TX slot being released in Counters::tx_stuck (also
 *     folded into Status::timeouts), and the record ends as FAIL (or WAIT when
 *     the whole bus stayed silent) instead of PASS. Retest such a board after a
 *     reset; nothing here calls Error_Handler() or blocks, so the other tests
 *     keep running.
 *   - PASS cannot come from stale credit: besides the cumulative minimum counts
 *     it requires a clean 2 s window, and a timeout is recorded as soon as a
 *     peer/class pair or the own TX FIFO makes no progress for 200 ms. A record
 *     that says PASS therefore had continuously verified bidirectional traffic
 *     for the whole window, not just once at the beginning.
 *   - tx counts released TX FIFO slots; the actual on-bus proof is the peer's
 *     verified count. The two are reported separately on purpose.
 *   - The module does not measure electrical parameters (termination value,
 *     bus levels) or the sample point; it only exercises the digital path at
 *     1 Mbit/s nominal / 5 Mbit/s data with the two frame classes.
 *   - Reports are live snapshots of separate volatile counters, taken without
 *     stopping the bus, so a single SWD read can show a frame in `rx` before it
 *     appears in `good`.
 *   - Generated code owns hardware initialization and IRQ dispatch. Runtime
 *     peer filters, TDC and receive callbacks use bsp::can::Can exclusively;
 *     public HAL getters provide FIFO completion, bus-off and TEC/REC because
 *     the package does not expose diagnostics. No register access is used.
 *   - FIFO completion attribution requires the generated TX FIFO mode and
 *     eight elements, with the package's software TX queue disabled. Submission
 *     alone never increases tx. No abort API is used; released slots are
 *     corroborated by both peers' independently verified frame counters.
 */
#pragma once

#include "test_common.hpp"

#include <cstddef>
#include <cstdint>

namespace fulltest::can
{
/// Number of controllers wired to the same bus and reported by statuses().
inline constexpr std::size_t controller_count = 3;

/**
 * Per-controller live counters, in addition to the three shared Status records.
 * Fields written by the receive interrupt are marked RX, fields written by
 * poll() are marked TX. Arrays are indexed by source controller (0..2); the
 * entry of the controller itself stays 0 because its own frames are filtered
 * out in hardware.
 */
struct Counters
{
    // RX (written in this controller's receive interrupt)
    volatile std::uint32_t rx_classic{0}; ///< classic frames delivered by the filter.
    volatile std::uint32_t rx_fd{0};      ///< FD frames delivered by the filter.
    volatile std::uint32_t verified_classic[controller_count]{}; ///< classic frames fully verified per source.
    volatile std::uint32_t verified_fd[controller_count]{};      ///< FD frames fully verified per source.
    volatile std::uint32_t mismatch_classic{0}; ///< classic content/sequence failures.
    volatile std::uint32_t mismatch_fd{0};      ///< FD content/sequence failures.
    volatile std::uint32_t foreign{0};          ///< frames outside the expected peer IDs.
    volatile std::uint32_t last_seq_classic[controller_count]{}; ///< last accepted sequence per source.
    volatile std::uint32_t last_seq_fd[controller_count]{};      ///< last accepted sequence per source.
    volatile std::uint32_t last_rx_ms[controller_count][2]{};    ///< [source][0 classic, 1 FD] tick of the last verified frame, 0 = never.

    // TX (written by poll())
    volatile std::uint32_t tx_classic{0}; ///< classic frames confirmed out of the TX FIFO.
    volatile std::uint32_t tx_fd{0};      ///< FD frames confirmed out of the TX FIFO.
    volatile std::uint32_t submitted_classic{0}; ///< classic frames accepted by send().
    volatile std::uint32_t submitted_fd{0};      ///< FD frames accepted by send().
    volatile std::uint32_t submit_busy{0};       ///< send() reported Busy (TX FIFO full).
    volatile std::uint32_t submit_error{0};      ///< send() reported any other failure.
    volatile std::uint32_t tx_stuck{0};          ///< 200 ms windows in which no TX FIFO slot was released while frames were pending.
    volatile std::uint32_t tec{0};               ///< FDCAN transmit error counter (live).
    volatile std::uint32_t rec{0};               ///< FDCAN receive error counter (live).
    volatile std::uint32_t busoff{0};            ///< bus-off observations (latched once).
    volatile std::uint32_t init_error{0};        ///< init/start failure reason code, 0 = none.
};

/// Use the already-generated HAL handles, configure filters/TDC through the
/// CANDriver2 package, register receive callbacks and start the buses. Never
/// blocks on the bus or calls Error_Handler(): a controller that fails to start
/// is reported as FAIL while the others keep running. Call once before poll().
void init();

/// Drive the periodic transmissions, collect TX completion, watchdog each
/// peer/class and re-evaluate the three records. Non-blocking; call every few
/// milliseconds, for example from the 1 ms poll loop.
void poll(std::uint32_t now_ms);

/// Three records, one per controller (CAN1, CAN2, CAN3 in order). The count is
/// written before the pointer is returned; both stay valid forever.
const Status* statuses(std::size_t& count) noexcept;

/// The per-controller counters behind the records, same order as statuses().
const Counters* counters(std::size_t& count) noexcept;
} // namespace fulltest::can

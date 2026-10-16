/**
 * @file    can_test.cpp
 * @author  syhanjin
 * @date    2026-10-15
 * @brief   CAN1/CAN2/CAN3 cross-controller bring-up test; see can_test.hpp for
 *          the wiring, protocol, judging rules and record semantics.
 *
 * Hardware configuration belongs to the CubeMX .ioc and generated fdcan.c:
 * three disjoint message RAM windows at 0 / 512 / 1024 words, four standard
 * filters per controller, automatic retransmission, 1 Mbit/s nominal and
 * 5 Mbit/s data, eight 64-byte RX FIFO0 and TX FIFO elements, and both IRQ
 * lines at priority 5. UserCode neither reinitializes handles nor owns IRQs.
 * Only peer-ID/global filtering, TX delay compensation and receive callbacks
 * are configured here through bsp::can::Can before start().
 *
 * The module never blocks on the bus, never sends a motor command (IDs here are
 * 0x300..0x302 and 0x310..0x312; the motor IDs used by the old firmware were
 * 0x000..0x00F and 0x1FF..0x20x) and never calls Error_Handler().
 */
#include "can_test.hpp"

#include "can_driver.hpp" /* bsp::can, BasicComponents bsp/can_driver2 */
#include "crc.hpp"
#include "fdcan.h"
#include "main.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace fulltest::can
{
namespace
{
namespace bc = bsp::can;

/// SSP offset in mtq: ~109 ns after the loopback delay the hardware measures.
constexpr std::uint32_t kTdcOffsetMtq = 12U;
constexpr std::uint32_t kTdcFilterMtq = 0U;

// ---------------------------------------------------------------------------
// Frame protocol (see can_test.hpp).
// ---------------------------------------------------------------------------
constexpr std::uint32_t kClassicIdBase = 0x300U; ///< + source index.
constexpr std::uint32_t kFdIdBase      = 0x310U; ///< + source index.
constexpr std::uint8_t  kMagic         = 0xA5U;
constexpr std::size_t   kClassicLength = 8U;
constexpr std::size_t   kFdLength      = 64U;

constexpr std::size_t   kClassic = 0U;
constexpr std::size_t   kFd      = 1U;
constexpr std::size_t   kClassCount = 2U;

constexpr std::uint32_t kClassicPeriodMs = 20U;
constexpr std::uint32_t kFdPeriodMs      = 40U;
constexpr std::uint32_t kClassicPhaseMs[controller_count] = { 0U, 3U, 6U };
constexpr std::uint32_t kFdPhaseMs[controller_count]      = { 0U, 5U, 10U };

/// Judgement thresholds and windows, documented in can_test.hpp.
constexpr std::uint32_t kMinTxCompletedPerClass  = 8U;
constexpr std::uint32_t kMinVerifiedPerPeerClass = 8U;
constexpr std::uint32_t kStarveMs                = 200U;
constexpr std::uint32_t kQuietMs                 = 2000U;
constexpr std::uint32_t kFailAfterMs             = 8000U;

/// Number of peer records one controller must hear from, i.e. the other
/// controllers on the shared bus. A controller's own verified counters stay 0
/// because its ID filters reject its own frames, so every coverage, freshness
/// and judging rule compares against this count and skips the own index.
constexpr std::size_t   kPeerCount = controller_count - 1U;
static_assert(kPeerCount >= 1U, "the CAN test needs at least one peer controller");

/// Status::detail bits.
constexpr std::uint32_t kDetailTxClassicDone = 1U << 0;
constexpr std::uint32_t kDetailTxFdDone      = 1U << 1;
constexpr std::uint32_t kDetailRxClassicBoth = 1U << 2;
constexpr std::uint32_t kDetailRxFdBoth      = 1U << 3;
/// Module-wide: no controller ever verified a peer frame, so the bus as a whole
/// stayed silent (no harness, no termination or no working transceiver).
constexpr std::uint32_t kDetailBusSilent     = 1U << 16;

/// Init/start failure reason codes, reported in Counters::init_error.
// Values 1/2 are unused; retain the existing SWD reason codes for driver setup.
constexpr std::uint32_t kInitErrorGlobal    = 3U;
constexpr std::uint32_t kInitErrorIdFilter  = 4U;
constexpr std::uint32_t kInitErrorTdc       = 5U;
constexpr std::uint32_t kInitErrorCallback  = 6U;
constexpr std::uint32_t kInitErrorStart     = 7U;

constexpr std::uint32_t kTxFifoCapacity = 8U; ///< TX FIFO capacity configured in the .ioc.
static_assert(bc::Can::TxQueueCapacity == 0U,
              "FIFO completion accounting requires no driver software TX queue");
using FrameCrc = crc::CRCX<8, 0x07, 0x00, false, false, 0x00>;

const char* const kNames[controller_count] = { "CAN1", "CAN2", "CAN3" };

// ---------------------------------------------------------------------------
// Filescope state. The Status and Counters records live in their own arrays so
// statuses()/counters() can hand out plain contiguous pointers.
// ---------------------------------------------------------------------------
Status   g_status[controller_count];
Counters g_counters[controller_count];

bc::Can              bus_can1(hfdcan1);
bc::Can              bus_can2(hfdcan2);
bc::Can              bus_can3(hfdcan3);
bc::Can* const       bus_list[controller_count]    = { &bus_can1, &bus_can2, &bus_can3 };
FDCAN_HandleTypeDef* const handle_list[controller_count] = { &hfdcan1, &hfdcan2, &hfdcan3 };

struct Controller
{
    bc::Can*             bus{ nullptr };
    FDCAN_HandleTypeDef* handle{ nullptr };
    Status*              status{ nullptr };
    Counters*            counters{ nullptr };
    std::size_t          index{ 0U };
    bool                 started{ false };

    /// Sequence numbers this controller stamps into its own frames.
    std::uint8_t tx_seq_classic{ 0U };
    std::uint8_t tx_seq_fd{ 0U };

    /// TX FIFO accounting: submission order is FIFO order (TxFifoQueueMode =
    /// FDCAN_TX_FIFO_OPERATION), so released slots are attributed in order.
    std::array<std::uint32_t, kClassCount> submitted{};
    std::array<std::uint32_t, kClassCount> completed{};
    std::array<std::uint8_t, kTxFifoCapacity> pending_class{};
    std::size_t pending_head{ 0U };
    std::size_t pending_count{ 0U };

    std::array<std::uint32_t, kClassCount> next_ms{};
    std::uint32_t start_ms{ 0U };
    std::uint32_t last_timeout_ms{ 0U };
    /// Tick at which a TX FIFO slot was released last, used to report a stuck
    /// transmitter (no ACK, bus-off) instead of an idle bus.
    std::uint32_t last_tx_progress_ms{ 0U };
    /// Tick of the last recorded trouble (mismatch/error/timeout), 0 = clean so
    /// far. Written by poll() and by the receive interrupt.
    volatile std::uint32_t last_trouble_ms{ 0U };
    bool busoff_seen{ false };

    /// First frame of a (class, source) pair skips the sequence check.
    /// Only touched inside the receive interrupt.
    std::array<std::array<bool, controller_count>, kClassCount> seen{};
};

Controller g_controllers[controller_count];

struct Receiver
{
    Controller* controller{ nullptr };
    void        on_frame(const bc::FrameView& frame) noexcept
    {
        verify_frame(*controller, frame);
    }

    static void verify_frame(Controller& controller, const bc::FrameView& frame) noexcept;
};

Receiver g_receivers[controller_count];

// ---------------------------------------------------------------------------
// Frame payload helpers.
// ---------------------------------------------------------------------------
std::uint8_t pattern_byte(std::uint8_t source, std::uint8_t kind, std::uint8_t seq,
                          std::uint8_t index) noexcept
{
    return static_cast<std::uint8_t>(static_cast<std::uint8_t>(source * 0x37U) ^
                                     static_cast<std::uint8_t>(kind * 0x5BU) ^
                                     static_cast<std::uint8_t>(seq * 0x11U) ^
                                     static_cast<std::uint8_t>(index * 0x1DU) ^ kMagic);
}

void build_payload(std::uint8_t* out, std::size_t length, std::uint8_t source, std::uint8_t kind,
                   std::uint8_t seq) noexcept
{
    out[0] = kMagic;
    out[1] = source;
    out[2] = kind;
    out[3] = seq;
    out[4] = static_cast<std::uint8_t>(length);
    for (std::size_t index = 5U; index + 1U < length; ++index)
        out[index] = pattern_byte(source, kind, seq, static_cast<std::uint8_t>(index));
    out[length - 1U] = FrameCrc::calc(out, length - 1U);
}

// ---------------------------------------------------------------------------
// Receive path (runs in the FDCAN Rx interrupt of its own controller; the
// three FDCAN interrupts share one priority, so no two of them nest).
// ---------------------------------------------------------------------------
void reject(Controller& controller, std::size_t kind, std::uint32_t now) noexcept
{
    if (kind == kFd)
        controller.counters->mismatch_fd += 1U;
    else
        controller.counters->mismatch_classic += 1U;
    controller.last_trouble_ms = now;
}

void Receiver::verify_frame(Controller& controller, const bc::FrameView& frame) noexcept
{
    const std::uint32_t now      = HAL_GetTick();
    Counters&           counters = *controller.counters;

    // 1) Map the ID to a peer and a class. The filters let exactly the four
    //    peer IDs through, so anything else is unexpected traffic; a frame
    //    never maps to this controller itself (its own IDs are not accepted).
    std::size_t source = controller_count;
    std::size_t kind   = kClassCount;
    for (std::size_t candidate = 0U; candidate < controller_count && source == controller_count;
         ++candidate)
    {
        if (candidate == controller.index)
            continue;
        if (frame.header.id == kClassicIdBase + candidate)
        {
            source = candidate;
            kind   = kClassic;
        }
        else if (frame.header.id == kFdIdBase + candidate)
        {
            source = candidate;
            kind   = kFd;
        }
    }
    if (source == controller_count)
    {
        counters.foreign += 1U;
        return;
    }

    const bool is_fd = kind == kFd;
    (is_fd ? counters.rx_fd : counters.rx_classic) += 1U;

    // 2) The frame shape must match the class its ID announced, including BRS:
    //    an 8-byte "FD" frame or a non-BRS 64-byte frame is a mismatch, not a
    //    valid result.
    const std::size_t length   = is_fd ? kFdLength : kClassicLength;
    const bool        shape_ok = frame.header.id_type == bc::IdType::Standard &&
                          frame.header.type == bc::FrameType::Data &&
                          frame.header.format ==
                                  (is_fd ? bc::FrameFormat::Fd : bc::FrameFormat::Classic) &&
                          frame.header.bit_rate_switch == is_fd &&
                          frame.data.size() == length;
    if (!shape_ok)
    {
        reject(controller, kind, now);
        return;
    }

    // 3) Fixed fields, pattern and CRC-8.
    const std::uint8_t* const payload         = frame.data.data();
    const std::uint8_t        expected_source = static_cast<std::uint8_t>(source + 1U);
    if (payload[0] != kMagic || payload[1] != expected_source ||
        payload[2] != static_cast<std::uint8_t>(kind) ||
        payload[4] != static_cast<std::uint8_t>(length))
    {
        reject(controller, kind, now);
        return;
    }
    for (std::size_t index = 5U; index + 1U < length; ++index)
    {
        if (payload[index] != pattern_byte(expected_source, static_cast<std::uint8_t>(kind),
                                           payload[3], static_cast<std::uint8_t>(index)))
        {
            reject(controller, kind, now);
            return;
        }
    }
    if (payload[length - 1U] != FrameCrc::calc(payload, length - 1U))
    {
        reject(controller, kind, now);
        return;
    }

    // 4) Sequence per (source, class): a gap means a frame was lost, so it is
    //    counted as a mismatch and the counter resynchronises to the newest
    //    frame instead of staying behind forever.
    volatile std::uint32_t* const last_seq = is_fd ? counters.last_seq_fd : counters.last_seq_classic;
    if (controller.seen[kind][source])
    {
        const std::uint8_t expected = static_cast<std::uint8_t>(last_seq[source] + 1U);
        if (payload[3] != expected)
        {
            last_seq[source] = payload[3];
            reject(controller, kind, now);
            return;
        }
    }
    else
    {
        controller.seen[kind][source] = true;
    }
    last_seq[source] = payload[3];

    if (is_fd)
        counters.verified_fd[source] += 1U;
    else
        counters.verified_classic[source] += 1U;
    counters.last_rx_ms[source][kind] = now;
}

// ---------------------------------------------------------------------------
// Transmit path (poll() context only).
// ---------------------------------------------------------------------------
void push_pending(Controller& controller, std::uint8_t kind) noexcept
{
    const std::size_t tail =
            (controller.pending_head + controller.pending_count) % kTxFifoCapacity;
    controller.pending_class[tail] = kind;
    controller.pending_count += 1U;
}

void transmit(Controller& controller, std::size_t kind, std::uint32_t now) noexcept
{
    // Do not overwrite an outstanding tag if hardware freed a slot since the
    // last observation. The next poll collects that completion before sending.
    if (controller.pending_count == kTxFifoCapacity)
        return;

    std::array<std::uint8_t, kFdLength> payload{};
    const std::size_t                   length = kind == kFd ? kFdLength : kClassicLength;
    std::uint8_t&                       seq =
            kind == kFd ? controller.tx_seq_fd : controller.tx_seq_classic;

    build_payload(payload.data(), length, static_cast<std::uint8_t>(controller.index + 1U),
                  static_cast<std::uint8_t>(kind), seq);

    bc::FrameHeader header{};
    header.id       = (kind == kFd ? kFdIdBase : kClassicIdBase) +
                static_cast<std::uint32_t>(controller.index);
    header.id_type  = bc::IdType::Standard;
    header.format   = kind == kFd ? bc::FrameFormat::Fd : bc::FrameFormat::Classic;
    header.type     = bc::FrameType::Data;
    header.bit_rate_switch = kind == kFd;

    const bc::FrameView frame{ header,
                               std::span<const std::uint8_t>(payload.data(), length) };

    switch (controller.bus->send(frame))
    {
    case bc::SendResult::Submitted:
        controller.submitted[kind] += 1U;
        if (kind == kFd)
            controller.counters->submitted_fd += 1U;
        else
            controller.counters->submitted_classic += 1U;
        push_pending(controller, static_cast<std::uint8_t>(kind));
        ++seq;
        break;
    case bc::SendResult::Busy:
        // TX FIFO full: the bus is not draining (no ACK, bus-off or heavy
        // retransmission). Nothing blocks here, the record just shows it.
        controller.counters->submit_busy += 1U;
        controller.last_trouble_ms = now;
        break;
    default:
        // NotStarted/InvalidFrame/Unsupported/BusOff/HardwareError. The driver
        // already validated the frame shape; these are real faults.
        controller.counters->submit_error += 1U;
        controller.last_trouble_ms = now;
        break;
    }
}

/// Attribute TX FIFO slots that were released since the last poll, and report a
/// TX FIFO that makes no progress. A released slot means the frame left the
/// FIFO, i.e. transmission completed on an acknowledged bus (or the frame was
/// aborted); the on-bus proof stays the peer's verified counter, which is
/// reported separately.
///
/// The driver exposes no abort/cancel API, so a frame that cannot be
/// acknowledged keeps occupying the head of the hardware TX FIFO and the frames
/// behind it are not transmitted at all. That condition is not silent here: as
/// long as frames are pending and no slot has been released for a whole window,
/// one stuck window is counted (Counters::tx_stuck) and added to the record's
/// timeout count, so a bus without ACK can never look like an idle bus and can
/// never contribute to PASS.
void collect_completion(Controller& controller, std::uint32_t now) noexcept
{
    const std::uint32_t free_slots = HAL_FDCAN_GetTxFifoFreeLevel(controller.handle);
    const std::uint32_t used =
            free_slots >= kTxFifoCapacity ? 0U : kTxFifoCapacity - free_slots;
    const std::uint32_t outstanding = (controller.submitted[kClassic] + controller.submitted[kFd]) -
                                      (controller.completed[kClassic] + controller.completed[kFd]);

    if (outstanding > used)
    {
        std::uint32_t finished = outstanding - used;
        while (finished > 0U && controller.pending_count > 0U)
        {
            const std::uint8_t kind = controller.pending_class[controller.pending_head];
            controller.pending_head = (controller.pending_head + 1U) % kTxFifoCapacity;
            controller.pending_count -= 1U;
            controller.completed[kind] += 1U;
            if (kind == kFd)
                controller.counters->tx_fd += 1U;
            else
                controller.counters->tx_classic += 1U;
            finished -= 1U;
        }
        controller.last_tx_progress_ms = now; // the FIFO made progress in this poll
        return;
    }

    // Nothing left the FIFO this poll while submitted frames are still pending:
    // the head frame is stuck (no ACK, bus-off or arbitration starvation).
    if (outstanding > 0U && elapsed(now, controller.last_tx_progress_ms, kStarveMs))
    {
        controller.last_tx_progress_ms = now;
        controller.counters->tx_stuck += 1U;
        controller.last_trouble_ms = now;
        // One deadline per window, shared with the receive starvation detector:
        // a dead bus must not inflate the printed timeout count twice.
        if (elapsed(now, controller.last_timeout_ms, kStarveMs))
        {
            controller.last_timeout_ms = now;
            controller.status->timeouts += 1U;
        }
    }
}

void collect_diagnostics(Controller& controller, std::uint32_t now) noexcept
{
    FDCAN_ProtocolStatusTypeDef protocol{};
    if (HAL_FDCAN_GetProtocolStatus(controller.handle, &protocol) == HAL_OK)
    {
        if (protocol.BusOff != 0U && !controller.busoff_seen)
        {
            controller.busoff_seen = true;
            controller.counters->busoff += 1U;
            controller.last_trouble_ms = now;
        }
    }

    FDCAN_ErrorCountersTypeDef error_counters{};
    if (HAL_FDCAN_GetErrorCounters(controller.handle, &error_counters) == HAL_OK)
    {
        controller.counters->tec = error_counters.TxErrorCnt;
        controller.counters->rec = error_counters.RxErrorCnt;
    }

    // Starvation: every peer has to deliver both classes. A missing peer or
    // class is reported once per window instead of flooding the record.
    bool starved = false;
    for (std::size_t source = 0U; source < controller_count && !starved; ++source)
    {
        if (source == controller.index)
            continue;
        for (std::size_t kind = 0U; kind < kClassCount; ++kind)
        {
            const std::uint32_t last = controller.counters->last_rx_ms[source][kind];
            if (elapsed(now, last != 0U ? last : controller.start_ms, kStarveMs))
            {
                starved = true;
                break;
            }
        }
    }
    if (starved && elapsed(now, controller.last_timeout_ms, kStarveMs))
    {
        controller.last_timeout_ms = now;
        controller.status->timeouts += 1U;
        controller.last_trouble_ms = now;
    }
}

/// True as soon as any controller verified a frame from another controller.
/// A silent bus is indistinguishable from missing wiring/termination, so it is
/// reported as WAIT instead of blaming the board with FAIL, while the bus-off
/// and timeout counters stay visible in the record.
bool bus_observed() noexcept
{
    for (std::size_t receiver = 0U; receiver < controller_count; ++receiver)
    {
        for (std::size_t source = 0U; source < controller_count; ++source)
        {
            if (source == receiver)
                continue;
            for (std::size_t kind = 0U; kind < kClassCount; ++kind)
            {
                if (g_counters[receiver].last_rx_ms[source][kind] != 0U)
                    return true;
            }
        }
    }
    return false;
}

void evaluate(Controller& controller, std::uint32_t now) noexcept
{
    Status&   status   = *controller.status;
    Counters& counters = *controller.counters;

    // Only the other controllers are counted: the own index is skipped here and
    // in every other coverage/freshness rule, because its verified counters are
    // permanently 0 (own frames are rejected by this controller's filters) and
    // requiring them would make PASS unreachable.
    std::uint32_t peers_classic = 0U;
    std::uint32_t peers_fd      = 0U;
    std::uint32_t good          = 0U;
    for (std::size_t source = 0U; source < controller_count; ++source)
    {
        if (source == controller.index)
            continue;
        const std::uint32_t classic = counters.verified_classic[source];
        const std::uint32_t fd      = counters.verified_fd[source];
        good += classic + fd;
        if (classic >= kMinVerifiedPerPeerClass)
            ++peers_classic;
        if (fd >= kMinVerifiedPerPeerClass)
            ++peers_fd;
    }

    const bool tx_classic_done = controller.completed[kClassic] >= kMinTxCompletedPerClass;
    const bool tx_fd_done      = controller.completed[kFd] >= kMinTxCompletedPerClass;
    const bool rx_classic_done = peers_classic == kPeerCount;
    const bool rx_fd_done      = peers_fd == kPeerCount;

    status.tx     = controller.completed[kClassic] + controller.completed[kFd];
    status.rx     = counters.rx_classic + counters.rx_fd;
    status.good   = good;
    status.bad    = counters.mismatch_classic + counters.mismatch_fd;
    status.errors = counters.submit_error + counters.busoff +
                    (counters.init_error != 0U ? 1U : 0U);

    std::uint32_t detail = 0U;
    if (tx_classic_done)
        detail |= kDetailTxClassicDone;
    if (tx_fd_done)
        detail |= kDetailTxFdDone;
    if (rx_classic_done)
        detail |= kDetailRxClassicBoth;
    if (rx_fd_done)
        detail |= kDetailRxFdBoth;
    detail |= (peers_classic & 0x3U) << 4;
    detail |= (peers_fd & 0x3U) << 8;
    detail |= (counters.init_error & 0xFU) << 12;

    const bool silent = !bus_observed();
    if (silent)
        detail |= kDetailBusSilent;
    status.detail = detail;

    const bool trouble_recent =
            controller.last_trouble_ms != 0U && !elapsed(now, controller.last_trouble_ms, kQuietMs);
    const bool expired = elapsed(now, controller.start_ms, kFailAfterMs);

    if (tx_classic_done && tx_fd_done && rx_classic_done && rx_fd_done &&
        !controller.busoff_seen && !trouble_recent)
        status.state = State::Pass;
    else if (expired && silent)
        // Nobody on the bus ever heard anybody: the harness/topology is absent
        // rather than this controller being broken. No traffic was verified, so
        // this is NOT a pass, and the error/timeout counters stay visible.
        status.state = State::Waiting;
    else if (controller.busoff_seen || trouble_recent || expired)
        status.state = State::Fail;
    else
        status.state = State::Running;
}

void fail_init(Controller& controller, std::uint32_t reason) noexcept
{
    controller.started = false;
    controller.counters->init_error = reason;
    controller.status->state        = State::Fail;
    controller.status->errors       = 1U;
}

} // namespace

// ---------------------------------------------------------------------------
// Public API.
// ---------------------------------------------------------------------------
void init()
{
    const std::uint32_t now = HAL_GetTick();

    for (std::size_t i = 0U; i < controller_count; ++i)
    {
        Controller& controller = g_controllers[i];
        controller.index       = i;
        controller.bus         = bus_list[i];
        controller.handle      = handle_list[i];
        controller.status      = &g_status[i];
        controller.counters    = &g_counters[i];
        g_receivers[i].controller = &controller;

        controller.status->name  = kNames[i];
        controller.status->state = State::Starting;
        controller.start_ms      = now;
        controller.next_ms[kClassic] = now + kClassicPhaseMs[i];
        controller.next_ms[kFd]      = now + kFdPhaseMs[i];
        controller.last_timeout_ms   = now;
        controller.last_tx_progress_ms = now;

        // Reject non-peer traffic; accept exactly the four peer/class IDs.
        const bc::GlobalFilter global_filter{ .standard                 = bc::NonMatchingAction::Reject,
                                              .extended                 = bc::NonMatchingAction::Reject,
                                              .reject_standard_remote   = true,
                                              .reject_extended_remote   = true };
        if (controller.bus->configure_filter(global_filter) != bc::Status::Ok)
        {
            fail_init(controller, kInitErrorGlobal);
            continue;
        }

        std::uint32_t element    = 0U;
        bool          filters_ok = true;
        for (std::size_t source = 0U; source < controller_count && filters_ok; ++source)
        {
            if (source == i)
                continue;
            for (std::size_t kind = 0U; kind < kClassCount; ++kind)
            {
                bc::IdFilter filter{};
                filter.index   = element++;
                filter.id_type = bc::IdType::Standard;
                filter.mode    = bc::FilterMode::Mask;
                filter.action  = bc::FilterAction::Fifo0;
                filter.id1     = (kind == kFd ? kFdIdBase : kClassicIdBase) + source;
                filter.id2     = 0x7FFU; // every identifier bit must match
                if (controller.bus->configure_filter(filter) != bc::Status::Ok)
                {
                    filters_ok = false;
                    break;
                }
            }
        }
        if (!filters_ok)
        {
            fail_init(controller, kInitErrorIdFilter);
            continue;
        }

        // Transmitter delay compensation for the 5 Mbit/s data phase.
        const bc::TxDelayCompensation tdc{ .enabled    = true,
                                          .offset_mtq = kTdcOffsetMtq,
                                          .filter_mtq = kTdcFilterMtq };
        if (controller.bus->configure_tx_delay_compensation(tdc) != bc::Status::Ok)
        {
            fail_init(controller, kInitErrorTdc);
            continue;
        }

        // Register the driver's receive callback, then start the generated handle.
        if (controller.bus->register_callback<&Receiver::on_frame>(g_receivers[i]) != bc::Status::Ok)
        {
            fail_init(controller, kInitErrorCallback);
            continue;
        }
        if (controller.bus->start() != bc::Status::Ok)
        {
            fail_init(controller, kInitErrorStart);
            continue;
        }

        controller.started = true;
        const std::uint32_t started_at = HAL_GetTick();
        controller.start_ms           = started_at;
        controller.last_timeout_ms    = started_at;
        controller.last_tx_progress_ms = started_at;
        controller.next_ms[kClassic]  = started_at + kClassicPhaseMs[i];
        controller.next_ms[kFd]       = started_at + kFdPhaseMs[i];
        controller.status->state      = State::Running;
    }
}

void poll(std::uint32_t now_ms)
{
    for (std::size_t i = 0U; i < controller_count; ++i)
    {
        Controller& controller = g_controllers[i];
        if (!controller.started)
            continue; // FAIL/START record is final for this controller.
        collect_completion(controller, now_ms);

        for (std::size_t kind = 0U; kind < kClassCount; ++kind)
        {
            const std::uint32_t period = kind == kFd ? kFdPeriodMs : kClassicPeriodMs;
            if (!elapsed(now_ms, controller.next_ms[kind], period))
                continue;
            controller.next_ms[kind] += period;
            if (elapsed(now_ms, controller.next_ms[kind], period))
                controller.next_ms[kind] = now_ms; // resynchronise after a stall
            transmit(controller, kind, now_ms);
        }

        collect_completion(controller, now_ms);
        collect_diagnostics(controller, now_ms);
        evaluate(controller, now_ms);
    }
}

const Status* statuses(std::size_t& count) noexcept
{
    count = controller_count;
    return g_status;
}

const Counters* counters(std::size_t& count) noexcept
{
    count = controller_count;
    return g_counters;
}
} // namespace fulltest::can

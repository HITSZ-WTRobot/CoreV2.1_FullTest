#include "uart_test.hpp"

#include "RingBuffer.hpp"
#include "UartRxSync.hpp"
#include "crc.hpp"
#include "memory.hpp"
#include "usart.h"

#include <array>
#include <cstring>

namespace fulltest::uart
{
namespace
{
constexpr uint8_t kStart = '$';
constexpr uint8_t kEnd = '\n';
constexpr char kKindRequest = 'R';
constexpr char kKindAnswer = 'A';
constexpr uint8_t kHostSrc = 0xFFu;
constexpr std::size_t kCrcOffset = 12;
constexpr std::size_t kCrcSpan = 11;
constexpr uint32_t kPeriodMs = 120;
constexpr uint32_t kResponseTimeoutMs = 60;
constexpr uint32_t kQuietGapMs = 6;
constexpr uint32_t kStartDelayMs = 60;
constexpr uint32_t kPassStreak = 3;
constexpr uint32_t kFailStreak = 5;
constexpr uint32_t kReceiverTimeoutMs = 500;
constexpr uint32_t kDetailSelfEcho = 1u << 0;
constexpr uint32_t kDetailLinkSeen = 1u << 1;
constexpr uint32_t kDetailHostSeen = 1u << 2;
constexpr uint32_t kDetailRingOverflow = 1u << 3;
constexpr uint32_t kDetailReceiverOnline = 1u << 4;
using CRC8 = crc::CRCX<8, 0x07, 0x00, false, false, 0x00>;

bool reached(uint32_t now, uint32_t deadline)
{
    return static_cast<int32_t>(now - deadline) >= 0;
}

template <typename T> void bump(volatile T& counter)
{
    counter = static_cast<T>(counter + 1u);
}

uint16_t pattern_for(uint16_t seq, uint8_t src)
{
    const uint32_t mixed = static_cast<uint32_t>(seq) * 2654435761u;
    return static_cast<uint16_t>((mixed >> 16) ^ mixed ^ (static_cast<uint32_t>(src) * 0x2B1Du));
}

uint8_t hex_digit(uint8_t value)
{
    return static_cast<uint8_t>(value < 10u ? ('0' + value) : ('A' + value - 10u));
}

int hex_value(uint8_t character)
{
    if (character >= '0' && character <= '9') return character - '0';
    if (character >= 'A' && character <= 'F') return character - 'A' + 10;
    if (character >= 'a' && character <= 'f') return character - 'a' + 10;
    return -1;
}

void put_hex(uint8_t* out, std::size_t offset, uint16_t value, std::size_t digits)
{
    for (std::size_t i = 0; i < digits; ++i)
        out[offset + digits - 1u - i] =
            hex_digit(static_cast<uint8_t>((static_cast<uint32_t>(value) >> (4u * i)) & 0xFu));
}

bool get_hex(const uint8_t* in, std::size_t offset, std::size_t digits, uint16_t& out)
{
    uint16_t value = 0;
    for (std::size_t i = 0; i < digits; ++i)
    {
        const int nibble = hex_value(in[offset + i]);
        if (nibble < 0) return false;
        value = static_cast<uint16_t>((value << 4) | static_cast<uint16_t>(nibble));
    }
    out = value;
    return true;
}

void encode(uint8_t* out, uint8_t src, uint16_t seq, uint16_t pat)
{
    out[0] = kStart;
    put_hex(out, 1, src, 2);
    out[3] = kKindRequest;
    put_hex(out, 4, seq, 4);
    put_hex(out, 8, pat, 4);
    put_hex(out, kCrcOffset, CRC8::calc(&out[1], kCrcSpan), 2);
    out[frame_length - 1u] = kEnd;
}

// Complete wire image and ISR arrival tick survive until the task consumes it.
struct Frame
{
    uint8_t bytes[frame_length];
    uint32_t at_ms;
    uint16_t seq;
    uint16_t pat;
    uint8_t src;
    char kind;
    bool valid;
};

bool decode_frame(Frame& frame)
{
    const uint8_t* in = frame.bytes;
    if (in[0] != kStart || in[frame_length - 1u] != kEnd) return false;
    if (in[3] != kKindRequest && in[3] != kKindAnswer) return false;
    uint16_t src = 0;
    uint16_t crc = 0;
    if (!get_hex(in, 1, 2, src) || !get_hex(in, 4, 4, frame.seq) ||
        !get_hex(in, 8, 4, frame.pat) || !get_hex(in, kCrcOffset, 2, crc))
        return false;
    if (crc != CRC8::calc(&in[1], kCrcSpan)) return false;
    frame.src = static_cast<uint8_t>(src);
    frame.kind = static_cast<char>(in[3]);
    return true;
}

class Receiver final : public protocol::UartRxSync<1, frame_length, true>
{
public:
    Receiver(UART_HandleTypeDef* uart, Buffer& buffer) : UartRxSync(uart, buffer) {}

    // One IRQ producer (UART/DMA IRQs share priority 5), one polling consumer.
    // Non-overwriting push(builder) publishes only fully constructed events.
    libs::RingBuffer<Frame, 5, false> events;
    volatile uint32_t received{0};
    volatile uint32_t malformed{0};
    volatile uint32_t overflows{0};
    volatile uint32_t errors{0};
    volatile uint32_t faults{0};
    volatile uint32_t last_hal_error{0};
    volatile uint32_t last_frame_ms{0};

protected:
    const std::array<uint8_t, 1>& header() const override
    {
        static constexpr std::array<uint8_t, 1> value{kStart};
        return value;
    }

    uint32_t timeout() const override { return kReceiverTimeoutMs; }

    bool decode(const uint8_t data[frame_length]) override
    {
        const uint32_t at = HAL_GetTick();
        last_frame_ms = at;
        bool valid = false;
        const bool queued = events.push([&](Frame& frame) {
            std::memcpy(frame.bytes, data, frame_length);
            frame.at_ms = at;
            frame.valid = decode_frame(frame);
            valid = frame.valid;
            if (valid)
                bump(received);
            else
            {
                bump(malformed);
                bump(faults);
            }
        });
        if (!queued)
        {
            bump(overflows);
            bump(faults);
        }
        return queued && valid;
    }
};

// DMABuffer supplies alignment, MEM_DMA supplies DMA-reachable non-cacheable
// AXI custom-section storage, explicitly initialized in init() (not startup-copied).
MEM_DMA memory::DMABuffer<frame_length> dma_buffers[port_count];
Receiver rx1(&huart1, dma_buffers[0]);
Receiver rx2(&huart2, dma_buffers[1]);
Receiver rx3(&huart3, dma_buffers[2]);
Receiver rx4(&huart4, dma_buffers[3]);
Receiver rx6(&huart6, dma_buffers[4]);
Receiver rx9(&huart9, dma_buffers[5]);
Receiver rx10(&huart10, dma_buffers[6]);

struct Port
{
    UART_HandleTypeDef* huart;
    const char* name;
    Receiver* receiver;
    std::size_t index;
    bool half_duplex;
    std::size_t peer_index;
    uint32_t phase_ms;
    bool ready{false};
    volatile bool tx_done{false};
    volatile bool tx_inflight{false};
    volatile uint32_t tx_completed{0};
    volatile uint32_t tx_at_ms{0};
    uint8_t tx_buffer[frame_length]{}; // IT TX, not DMA: ordinary RAM is correct.
    uint16_t sequence{0};
    uint16_t pending_pattern{0};
    bool awaiting{false};
    bool exchange_failed{false};
    bool rx_failed{false};
    bool rx_verified{false};
    bool tx_failure_recorded{false};
    bool ever_verified{false};
    uint32_t attempts{0};
    uint32_t good_streak{0};
    uint32_t bad_streak{0};
    uint32_t next_tx_ms{0};
    uint32_t started_ms{0};
    uint32_t deadline_ms{0};
    uint32_t exchange_faults{0};
    uint32_t seen_faults{0};
    uint32_t api_errors{0};
    uint32_t tx_failures{0};
    uint32_t tx_timeouts{0};
    uint32_t rejected{0};
    uint32_t flags{0};
};

Status records[port_count];
Port ports[port_count] = {
    {&huart1, "USART1", &rx1, 0, false, 3, 0},
    {&huart2, "USART2_RS485", &rx2, 1, true, 1, 60},
    {&huart3, "USART3", &rx3, 2, false, 5, 20},
    {&huart4, "UART4", &rx4, 3, false, 0, 0},
    {&huart6, "USART6", &rx6, 4, false, 6, 40},
    {&huart9, "UART9", &rx9, 5, false, 2, 20},
    {&huart10, "USART10", &rx10, 6, false, 4, 40},
};
bool initialised = false;

Port* port_for(UART_HandleTypeDef* huart)
{
    for (auto& port : ports)
        if (port.huart == huart) return &port;
    return nullptr;
}

void handle_rx_complete(UART_HandleTypeDef* huart)
{
    if (Port* port = port_for(huart)) port->receiver->receiveCallback();
}

void handle_tx_complete(UART_HandleTypeDef* huart)
{
    if (Port* port = port_for(huart))
    {
        port->tx_at_ms = HAL_GetTick();
        bump(port->tx_completed);
        port->tx_inflight = false;
        port->tx_done = true;
    }
}

void handle_error(UART_HandleTypeDef* huart)
{
    if (Port* port = port_for(huart))
    {
        Receiver& receiver = *port->receiver;
        receiver.last_hal_error = HAL_UART_GetError(huart);
        bump(receiver.errors);
        bump(receiver.faults);
        receiver.errorHandler();
    }
}

void failure_event(Port& port, bool receive_failure = true)
{
    port.good_streak = 0;
    ++port.bad_streak;
    if (!port.half_duplex) ports[port.peer_index].good_streak = 0;
    if (port.awaiting)
    {
        port.exchange_failed = true;
        if (receive_failure) port.rx_failed = true;
    }
}

bool response_candidate(const Port& port, const Port& source, const Frame& frame)
{
    if (port.half_duplex) return frame.kind == kKindAnswer && frame.src == kHostSrc;
    return frame.kind == kKindRequest && frame.src == static_cast<uint8_t>(source.index);
}

// False leaves the front slot owned by the queue while the source awaits TC.
bool evaluate(Port& port, const Frame& frame, uint32_t now)
{
    if (!frame.valid) return true; // Counted by the receiver and its fault epoch.
    if (port.half_duplex && frame.kind == kKindRequest &&
        frame.src == static_cast<uint8_t>(port.index))
    {
        port.flags |= kDetailSelfEcho;
        return true; // Never mistake the transceiver's local echo for a host answer.
    }
    if (port.half_duplex && frame.kind == kKindAnswer && frame.src == kHostSrc)
        port.flags |= kDetailHostSeen;
    const Port& source = ports[port.peer_index];
    if (!port.awaiting || port.rx_verified || !source.awaiting ||
        !response_candidate(port, source, frame) || frame.seq != source.sequence ||
        frame.pat != source.pending_pattern || !reached(frame.at_ms, source.started_ms) ||
        reached(frame.at_ms, source.deadline_ms))
    {
        ++port.rejected;
        failure_event(port);
        return true;
    }
    if (port.rx_failed || port.receiver->faults != port.exchange_faults) return true;
    if (!source.tx_done)
        return reached(now, source.deadline_ms); // Retain an early RX until TC/deadline.
    if (reached(source.tx_at_ms, source.deadline_ms) || source.tx_failure_recorded)
        return true;
    bump(records[port.index].good);
    port.flags |= kDetailLinkSeen;
    port.rx_verified = true;
    return true;
}

void prepare_exchange(Port& port, uint32_t now)
{
    ++port.attempts;
    ++port.sequence;
    port.pending_pattern = pattern_for(port.sequence, static_cast<uint8_t>(port.index));
    encode(port.tx_buffer, static_cast<uint8_t>(port.index), port.sequence, port.pending_pattern);
    port.tx_done = false;
    port.awaiting = true;
    port.exchange_failed = false;
    port.rx_failed = false;
    port.rx_verified = false;
    port.tx_failure_recorded = false;
    port.exchange_faults = port.receiver->faults;
    port.started_ms = now;
    port.deadline_ms = now + kResponseTimeoutMs;
    port.next_tx_ms = now + kPeriodMs;
}

void transmit(Port& port)
{
    port.tx_inflight = true;
    if (HAL_UART_Transmit_IT(port.huart, port.tx_buffer, frame_length) != HAL_OK)
    {
        port.tx_inflight = false;
        ++port.api_errors;
        ++port.tx_failures;
        port.tx_failure_recorded = true;
        failure_event(port, false);
    }
}

void poll_receiver(Port& port, uint32_t now)
{
    Receiver& receiver = *port.receiver;
    const uint32_t faults = receiver.faults;
    if (faults != port.seen_faults)
    {
        port.seen_faults = faults;
        failure_event(port);
    }
    // Never pop and retain its pointer: an IRQ producer could reuse that slot.
    while (const Frame* frame = receiver.events.front())
    {
        if (!evaluate(port, *frame, now)) break;
        (void)receiver.events.pop();
    }
}

bool round_verified(const Port& port)
{
    return port.rx_verified && !port.exchange_failed &&
           port.receiver->faults == port.exchange_faults;
}

void finish_exchange(Port& port, bool group_verified)
{
    if (!port.tx_failure_recorded &&
        (!port.tx_done || reached(port.tx_at_ms, port.deadline_ms)))
    {
        ++port.tx_failures;
        ++port.tx_timeouts;
        port.tx_failure_recorded = true;
        failure_event(port, false);
    }
    if (!port.rx_verified)
    {
        bump(records[port.index].timeouts);
        failure_event(port);
    }
    if (group_verified)
    {
        ++port.good_streak;
        port.bad_streak = 0;
        port.ever_verified = true;
    }
    else port.good_streak = 0;
    port.awaiting = false;
}

bool can_start(const Port& port, uint32_t now)
{
    // An overdue IT transmission still owns its buffer until the real TC IRQ.
    return port.ready && !port.awaiting && !port.tx_inflight &&
           reached(now, port.next_tx_ms) &&
           reached(now, port.receiver->last_frame_ms + kQuietGapMs);
}

void poll_group(Port& port, uint32_t now)
{
    Port& peer = ports[port.peer_index]; // USART2 deliberately refers to itself.
    if (port.awaiting && ((port.rx_verified && peer.rx_verified) ||
                          reached(now, port.deadline_ms)))
    {
        const bool verified = round_verified(port) && round_verified(peer);
        finish_exchange(port, verified);
        if (!port.half_duplex) finish_exchange(peer, verified);
    }
    if (!can_start(port, now) || !can_start(peer, now)) return;
    // Publish both independent source contexts before either UART can receive.
    prepare_exchange(port, now);
    if (!port.half_duplex) prepare_exchange(peer, now);
    transmit(port);
    if (!port.half_duplex) transmit(peer);
}

void publish_status(Port& port)
{
    Receiver& receiver = *port.receiver;
    const Port& peer = ports[port.peer_index];
    Status& status = records[port.index];
    status.tx = port.tx_completed;
    status.rx = receiver.received;
    status.bad = receiver.malformed + port.rejected;
    status.errors = port.api_errors + port.tx_timeouts + receiver.errors + receiver.overflows;
    if (!port.ready || !peer.ready) status.state = State::Fail;
    else if (port.attempts == 0) status.state = State::Starting;
    else if (port.good_streak >= kPassStreak && peer.good_streak >= kPassStreak)
        status.state = State::Pass;
    else if (port.bad_streak >= kFailStreak || peer.bad_streak >= kFailStreak)
        status.state = (port.ever_verified || peer.ever_verified) ? State::Fail : State::Waiting;
    else status.state = State::Running;
    uint32_t flags = port.flags;
    if (receiver.overflows != 0) flags |= kDetailRingOverflow;
    if (receiver.isConnected()) flags |= kDetailReceiverOnline;
    const uint32_t tx_failures = port.tx_failures < 0xFFFFu ? port.tx_failures : 0xFFFFu;
    status.detail = flags | ((receiver.last_hal_error & 0xFFu) << 8) | (tx_failures << 16);
}
} // namespace

void init()
{
    if (initialised) return;
    const uint32_t now = HAL_GetTick();
    for (auto& port : ports)
    {
        records[port.index].name = port.name;
        std::memset(dma_buffers[port.index].data(), 0, dma_buffers[port.index].size());
        port.receiver->last_frame_ms = now;
        port.next_tx_ms = now + kStartDelayMs + port.phase_ms;
        // Generated CubeMX code owns DMA, GPIO, DE timing, IRQs and callbacks
        // enablement. Register every callback before letting reception start.
        const bool rx_ok = HAL_UART_RegisterCallback(port.huart, HAL_UART_RX_COMPLETE_CB_ID,
                                                     handle_rx_complete) == HAL_OK;
        const bool tx_ok = HAL_UART_RegisterCallback(port.huart, HAL_UART_TX_COMPLETE_CB_ID,
                                                     handle_tx_complete) == HAL_OK;
        const bool error_ok = HAL_UART_RegisterCallback(port.huart, HAL_UART_ERROR_CB_ID,
                                                        handle_error) == HAL_OK;
        port.ready = rx_ok && tx_ok && error_ok && port.receiver->startReceive();
        if (!port.ready)
        {
            ++port.api_errors;
            records[port.index].errors = port.api_errors;
            records[port.index].state = State::Fail;
        }
    }
    initialised = true;
}

void poll(uint32_t now)
{
    if (!initialised) return;
    for (auto& port : ports) poll_receiver(port, now);
    for (auto& port : ports)
        if (port.half_duplex || port.index < port.peer_index) poll_group(port, now);
    for (auto& port : ports) publish_status(port);
}

const Status* statuses(std::size_t& count)
{
    count = port_count;
    return records;
}
} // namespace fulltest::uart

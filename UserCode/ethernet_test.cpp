/**
 * @file ethernet_test.cpp
 * @brief 不可变完整性响应与 ACK 驱动 DMA 报告分片；协议/接线见 ethernet_test.hpp。
 * 所有发送仅调用 cpkg UdpSocket，保留其借用 payload 的原生语义。
 */
#include "ethernet_test.hpp"

#include "dma_buffer.hpp"
#include "eth/eth.hpp"
#include "memory.hpp"
#include "udp_socket.hpp"

#include <crc.hpp>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <span>

namespace fulltest::ethernet
{
namespace
{
constexpr std::uint16_t kPort = 5001U;
constexpr std::size_t kLengths[]{8U, 64U, 256U, 512U, 1400U};
constexpr std::size_t kCaseCount = sizeof(kLengths) / sizeof(kLengths[0]);
constexpr char kAck[] = "CONFIRM ACK\n";
constexpr char kError[] = "ERR FAIL\n";
constexpr char kResultDone[] = "RESULT DONE\n";
constexpr char kResultBusy[] = "RESULT BUSY\n";
constexpr char kResultNotReady[] = "RESULT NOT_READY\n";
constexpr std::size_t kReportCapacity = 3072U;
constexpr std::size_t kReportChunk = 1100U;
constexpr std::size_t kReportHeader = 80U;
using ReportCRC = crc::CRCX<8, 0x07, 0x00, false, false, 0x00>;

// The immutable replies below are explicitly filled before opening the socket.
// Never rewrite them: asynchronous MAC/ARP queues may retain their pointers.
MEM_DMA memory::DMABuffer<8U> g_response8;
MEM_DMA memory::DMABuffer<64U> g_response64;
MEM_DMA memory::DMABuffer<256U> g_response256;
MEM_DMA memory::DMABuffer<512U> g_response512;
MEM_DMA memory::DMABuffer<1400U> g_response1400;
MEM_DMA memory::DMABuffer<sizeof(kAck) - 1U> g_ack;
MEM_DMA memory::DMABuffer<sizeof(kError) - 1U> g_error;
MEM_DMA memory::DMABuffer<sizeof(kResultDone) - 1U> g_result_done;
MEM_DMA memory::DMABuffer<sizeof(kResultBusy) - 1U> g_result_busy;
MEM_DMA memory::DMABuffer<sizeof(kResultNotReady) - 1U> g_result_not_ready;
// The only mutable TX allocation. A matching host ACK, never a timeout or a
// successful sendTo, proves that its one submitted datagram left DMA/ARP queues.
MEM_DMA memory::DMABuffer<kReportHeader + kReportChunk> g_result_tx;
const std::uint8_t* const g_responses[]{g_response8.data(), g_response64.data(),
                                      g_response256.data(), g_response512.data(),
                                      g_response1400.data()};

// RX alone may be overwritten on each poll; no send ever refers to this storage.
std::uint8_t g_rx[1472U];
net::UdpSocket g_socket;
Status g_status{"ETH"};
bool g_started = false;
bool g_socket_open = false;
bool g_link_up = false;
bool g_link_seen = false;
bool g_host_confirmed = false;
bool g_platform_failed = false;
bool g_session_active = false;
net::UdpEndpoint g_peer{};
std::uint32_t g_session_packets = 0U;
std::size_t g_next_case = 0U;
EthInit g_eth_init = EthInit::LwipTimeout;
std::uint32_t g_platform_error = 0U;
std::uint32_t g_last_errno = 0U;
std::uint32_t g_tx_base = 0U;

ReportProvider g_report_provider = nullptr;
char g_report_snapshot[kReportCapacity];
net::UdpEndpoint g_result_peer{};
std::uint32_t g_result_session = 0U;
std::uint32_t g_result_part = 0U;
std::uint32_t g_result_parts = 0U;
std::size_t g_result_total = 0U;
std::uint8_t g_result_crc = 0U;
bool g_result_waiting = false;

void add(volatile std::uint32_t& counter, const std::uint32_t value = 1U)
{
    counter = counter + value;
}

void socket_error(const int error)
{
    add(g_status.errors);
    g_last_errno = static_cast<std::uint32_t>(error);
}

std::uint8_t pattern(const std::size_t index, const std::size_t offset, const bool response)
{
    if (offset < 4U)
        return static_cast<std::uint8_t>((response ? "FTB1" : "FTA1")[offset]);
    if (offset == 4U)
        return static_cast<std::uint8_t>(index);
    const auto value = static_cast<std::uint8_t>(offset * 37U + index * 19U + 0x53U);
    return response ? static_cast<std::uint8_t>(value ^ 0xA5U) : value;
}

template <std::size_t N>
void fill_response(memory::DMABuffer<N>& buffer, const std::size_t index)
{
    for (std::size_t offset = 0U; offset < N; ++offset)
        buffer.data()[offset] = pattern(index, offset, true);
}

std::span<const std::byte> response(const std::size_t index)
{
    return {reinterpret_cast<const std::byte*>(g_responses[index]), kLengths[index]};
}

// Callers use permanent DMA storage: either immutable replies or the single
// report buffer frozen until this datagram's matching host ACK.
bool send(const net::UdpEndpoint& peer, const std::uint8_t* data, const std::size_t length)
{
    const int error = g_socket.sendTo(peer, {reinterpret_cast<const std::byte*>(data), length});
    if (error != 0)
    {
        socket_error(error);
        return false;
    }
    return true; // Submission only. Never used to increment Status::tx/good.
}

void reject(const net::UdpEndpoint& peer)
{
    add(g_status.bad);
    (void)send(peer, g_error.data(), g_error.size());
}

bool same_peer(const net::UdpEndpoint& peer)
{
    return (g_peer.port == peer.port) && ip4_addr_cmp(&g_peer.address, &peer.address);
}

bool faulted()
{
    return (g_eth_init != EthInit::Ok) || !g_socket_open || g_platform_failed ||
           (g_status.bad != 0U) || (g_status.errors != 0U);
}

// Strict decimal parsing with bounded arithmetic; no sscanf/temporary TX text.
bool parse_number(std::size_t& position, const std::size_t length,
                  const std::uint8_t delimiter, std::uint32_t& result)
{
    const std::size_t start = position;
    result = 0U;
    while ((position < length) && (g_rx[position] >= '0') && (g_rx[position] <= '9'))
    {
        const std::uint32_t digit = g_rx[position] - '0';
        if (result > (std::numeric_limits<std::uint32_t>::max() - digit) / 10U)
            return false;
        result = result * 10U + digit;
        ++position;
    }
    if ((position == start) || (position == length) || (g_rx[position] != delimiter))
        return false;
    ++position;
    return true;
}

void send_result_part()
{
    // Called only for a new session, or after the previous chunk's matching ACK.
    // There is no resend path, no completion-counter heuristic and no deadline.
    const std::size_t offset = static_cast<std::size_t>(g_result_part) * kReportChunk;
    const std::size_t remaining = g_result_total - offset;
    const std::size_t body_size = (remaining > kReportChunk) ? kReportChunk : remaining;
    const auto* body = reinterpret_cast<const std::uint8_t*>(g_report_snapshot + offset);
    g_result_crc = ReportCRC::calc(body, body_size);
    const int header_size = std::snprintf(
        reinterpret_cast<char*>(g_result_tx.data()), kReportHeader,
        "RESULT %lu %lu %lu %lu %u\n", static_cast<unsigned long>(g_result_session),
        static_cast<unsigned long>(g_result_part), static_cast<unsigned long>(g_result_parts),
        static_cast<unsigned long>(g_result_total), static_cast<unsigned>(g_result_crc));
    if ((header_size <= 0) || (static_cast<std::size_t>(header_size) >= kReportHeader))
    {
        // Nothing was submitted, so there is no borrowed mutable payload here.
        (void)send(g_result_peer, g_error.data(), g_error.size());
        return;
    }
    const auto header_length = static_cast<std::size_t>(header_size);
    std::memcpy(g_result_tx.data() + header_length, body, body_size);
    // Freeze BEFORE submission, including on sendTo failure: an API error is not
    // sufficient evidence that lower queues no longer retain the payload.
    g_result_waiting = true;
    (void)send(g_result_peer, g_result_tx.data(), header_length + body_size);
}

void request_result(const net::UdpEndpoint& peer, const std::uint32_t now)
{
    if (g_result_waiting)
    {
        (void)send(peer, g_result_busy.data(), g_result_busy.size());
        return;
    }
    if (g_report_provider == nullptr)
    {
        (void)send(peer, g_result_not_ready.data(), g_result_not_ready.size());
        return;
    }
    const auto report = g_report_provider(now);
    if (report.empty())
    {
        (void)send(peer, g_result_not_ready.data(), g_result_not_ready.size());
        return;
    }
    if ((report.size() > sizeof(g_report_snapshot)) || (g_result_session == UINT32_MAX))
    {
        // Never truncate the report or reuse a session number within one boot.
        (void)send(peer, g_error.data(), g_error.size());
        return;
    }
    std::memcpy(g_report_snapshot, report.data(), report.size());
    g_result_total = report.size();
    g_result_parts = static_cast<std::uint32_t>((g_result_total + kReportChunk - 1U) / kReportChunk);
    ++g_result_session;
    g_result_part = 0U;
    g_result_peer = peer;
    send_result_part();
}

void acknowledge_result(const std::size_t length, const net::UdpEndpoint& peer)
{
    std::size_t position = 5U; // "RACK " already matched.
    std::uint32_t session = 0U;
    std::uint32_t part = 0U;
    std::uint32_t checksum = 0U;
    const bool parsed = parse_number(position, length, ' ', session) &&
                        parse_number(position, length, ' ', part) &&
                        parse_number(position, length, '\n', checksum) && (position == length);
    if (!parsed || !g_result_waiting || (peer.port != g_result_peer.port) ||
        !ip4_addr_cmp(&peer.address, &g_result_peer.address) ||
        (session != g_result_session) || (part != g_result_part) || (checksum != g_result_crc))
        return; // Old, duplicate, malformed or foreign ACKs cannot release DMA storage.

    // The trusted host has received and verified this uniquely sent datagram.
    // Even after a send error we wait for this exact proof, with no timeout reuse.
    g_result_waiting = false;
    ++g_result_part;
    if (g_result_part == g_result_parts)
        (void)send(peer, g_result_done.data(), g_result_done.size());
    else
        send_result_part();
}

void confirm(const std::size_t length, const net::UdpEndpoint& peer)
{
    std::size_t position = 8U; // "CONFIRM " already matched.
    std::uint32_t rounds = 0U;
    std::uint32_t mismatch = 0U;
    const bool parsed = parse_number(position, length, ' ', rounds) &&
                        parse_number(position, length, '\n', mismatch) && (position == length);
    if (!parsed || !g_session_active || !same_peer(peer) || (mismatch != 0U) ||
        (rounds == 0U) || (rounds > UINT32_MAX / kCaseCount) || (g_next_case != 0U) ||
        (rounds * kCaseCount != g_session_packets) || faulted())
    {
        reject(peer);
        return;
    }

    // This is host feedback, not an inference from sendTo or whole-MAC events.
    add(g_status.good, g_session_packets);
    g_host_confirmed = true;
    g_session_active = false;
    g_session_packets = 0U;
    (void)send(peer, g_ack.data(), g_ack.size());
}

void handle_datagram(const net::UdpReceiveResult& received, const std::uint32_t now)
{
    const auto& peer = received.source;
    if (received.truncated)
    {
        reject(peer);
        return;
    }
    // Diagnostic sessions have their own peer/sequence and bypass integrity FAIL.
    // They must never advance the A-pattern/CONFIRM packet counters or ordering.
    if ((received.size == 7U) && (std::memcmp(g_rx, "RESULT\n", 7U) == 0))
    {
        request_result(peer, now);
        return;
    }
    if ((received.size >= 5U) && (std::memcmp(g_rx, "RACK ", 5U) == 0))
    {
        acknowledge_result(received.size, peer);
        return;
    }
    if ((received.size >= 8U) && (std::memcmp(g_rx, "CONFIRM ", 8U) == 0))
    {
        confirm(received.size, peer);
        return;
    }

    const std::size_t index = g_next_case;
    if ((received.size != kLengths[index]) ||
        (g_session_active && !same_peer(peer)) || (g_session_packets == UINT32_MAX))
    {
        reject(peer);
        return;
    }
    for (std::size_t offset = 0U; offset < received.size; ++offset)
    {
        if (g_rx[offset] != pattern(index, offset, false))
        {
            reject(peer);
            return;
        }
    }
    if (faulted())
    {
        reject(peer);
        return;
    }
    if (!g_session_active)
    {
        g_peer = peer;
        g_session_active = true;
    }
    const auto payload = response(index);
    if (send(peer, reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size()))
    {
        ++g_session_packets;
        g_next_case = (index + 1U) % kCaseCount;
    }
}

void update_status()
{
    if (faulted())
        g_status.state = State::Fail;
    else if (g_session_active)
        g_status.state = State::Running;
    else if (g_host_confirmed)
        g_status.state = State::Pass;
    else
        g_status.state = State::Waiting;

    std::uint32_t detail = 0U;
    if (g_eth_init == EthInit::Ok) detail |= 1U << 0U;
    if (g_link_up) detail |= 1U << 1U;
    if (g_socket_open) detail |= 1U << 2U;
    if (g_platform_failed) detail |= 1U << 3U;
    if (g_link_seen) detail |= 1U << 4U;
    if (g_host_confirmed) detail |= 1U << 5U;
    detail |= static_cast<std::uint32_t>(g_eth_init) << 8U;
    detail |= (g_platform_error & 0xFFU) << 16U;
    detail |= (g_last_errno & 0xFFU) << 24U;
    g_status.detail = detail;
}
} // namespace

void init()
{
    if (g_started)
        return;
    g_started = true; // Never reinitialize even if a TX buffer is still borrowed.
    fill_response(g_response8, 0U);
    fill_response(g_response64, 1U);
    fill_response(g_response256, 2U);
    fill_response(g_response512, 3U);
    fill_response(g_response1400, 4U);
    std::memcpy(g_ack.data(), kAck, g_ack.size());
    std::memcpy(g_error.data(), kError, g_error.size());
    std::memcpy(g_result_done.data(), kResultDone, g_result_done.size());
    std::memcpy(g_result_busy.data(), kResultBusy, g_result_busy.size());
    std::memcpy(g_result_not_ready.data(), kResultNotReady, g_result_not_ready.size());
    std::memset(g_result_tx.data(), 0, g_result_tx.size());

    g_tx_base = ETH_TxCompleteCount();
    g_eth_init = ETH_Init();
    if (g_eth_init == EthInit::Ok)
    {
        net::UdpEndpoint local{};
        local.port = kPort;
        const int error = g_socket.open(local);
        g_socket_open = (error == 0);
        if (!g_socket_open)
            socket_error(error);
    }
    else
        add(g_status.errors);
    update_status();
}

void set_report_provider(const ReportProvider provider) noexcept
{
    g_report_provider = provider;
}

void poll(const std::uint32_t now)
{
    if (!g_started)
        return;

    g_status.tx = ETH_TxCompleteCount() - g_tx_base;
    if (ETH_PlatformFailed())
    {
        if (!g_platform_failed)
            add(g_status.errors);
        g_platform_failed = true;
        g_platform_error = ETH_PlatformError();
    }
    g_link_up = ETH_LinkUp(); // netif snapshot only; never polls the PHY.
    g_link_seen = g_link_seen || g_link_up;

    if (g_socket_open)
    {
        const auto received = g_socket.receiveFrom(
            {reinterpret_cast<std::byte*>(g_rx), sizeof(g_rx)}, 0U);
        if (received.error == 0)
        {
            add(g_status.rx);
            update_status(); // The provider sees this poll's current link/platform status.
            handle_datagram(received, now);
        }
        else if ((received.error != EWOULDBLOCK) && (received.error != EAGAIN))
            socket_error(received.error);
    }
    update_status();
}

const Status* statuses(std::size_t& count)
{
    count = 1U;
    return &g_status;
}
} // namespace fulltest::ethernet

#pragma once

#include <cstddef>
#include <cstdint>

// Full-board manufacturing/bring-up test. No motor commands are generated.
// PASS requires observed feedback; WAIT means the required external loop is absent.
namespace fulltest
{
enum class State : uint32_t { Starting, Waiting, Running, Pass, Fail };

// Fixed storage, visible through SWD. Each module owns its records permanently.
// ISR-written fields are aligned 32-bit values; reports are live snapshots, not
// atomic transactions. Counters are cumulative and are never reset by reporting.
struct Status
{
    const char* name;
    volatile State state{State::Starting};
    volatile uint32_t tx{0};        // Module-specific TX/drive progress; not proof of delivery.
    volatile uint32_t rx{0};        // Received frames/transfers.
    volatile uint32_t good{0};      // Verified feedback/data.
    volatile uint32_t bad{0};       // Content/sequence mismatch.
    volatile uint32_t errors{0};    // Peripheral/API errors.
    volatile uint32_t timeouts{0};  // Feedback deadline expired.
    volatile uint32_t detail{0};    // Module-specific, documented beside record.
};

inline const char* state_name(State state)
{
    switch (state)
    {
    case State::Starting: return "START";
    case State::Waiting: return "WAIT";
    case State::Running: return "RUN";
    case State::Pass: return "PASS";
    case State::Fail: return "FAIL";
    }
    return "UNKNOWN";
}

// Wrap-safe for intervals less than 2^31 milliseconds.
inline bool elapsed(uint32_t now, uint32_t since, uint32_t interval)
{
    return static_cast<uint32_t>(now - since) >= interval;
}
} // namespace fulltest

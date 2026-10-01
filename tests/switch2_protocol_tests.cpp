// SPDX-License-Identifier: GPL-2.0-only
#include "../mc_mitm/source/controllers/switch2_protocol.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>

#ifdef NDEBUG
#error "These regression tests rely on assert() side effects; build without NDEBUG."
#endif

int main() {
    using namespace ams::controller::switch2;
    for (auto pid : {0x2060, 0x2061, 0x2062, 0x2064}) {
        const auto id = DefaultReportId(pid);
        const auto minimum = MinimumPayloadSize(id);
        assert(ValidInput(id, pid, minimum));
        assert(ValidInput(id, pid, 63)); // Regression: long defaults are NOT 0x05.
        assert(!ValidInput(id, pid, minimum - 1));
        assert(ValidInput(0x05, pid, 63));
        assert(!ValidInput(0x05, pid, 15));
        assert(!ValidInput(id, pid, 513));
    }
    assert(!ValidInput(0x08, 0x2060, 63));
    assert(!ValidInput(0x07, 0x2061, 63));
    assert(!ValidInput(0, 0x2060, 63));
    assert(!ValidInput(0x05, 0, 63));
    const std::uint8_t center[] = {0x00, 0x08, 0x80};
    const std::uint8_t maximum[] = {0xff, 0xff, 0xff};
    assert(StickX(center) == 2048 && StickY(center) == 2048);
    assert(StickX(maximum) == 4095 && StickY(maximum) == 4095);
    std::uint8_t response[] = {0x0C, 0x01, 0x01, 0x02, 0x10, 0x78, 0, 0};
    assert(IsCommandResponse(response, sizeof(response)));
    assert(!IsCommandResponse(response, 7));
    assert(!IsCommandResponse(nullptr, 8));
    assert(!IsCommandResponse(response, 513));
    response[1] = 0x91;
    assert(!IsCommandResponse(response, 8));
    response[1] = 0x01;
    response[2] = 0;
    assert(!IsCommandResponse(response, 8));
    assert(IsSuccessfulAck(0x78));
    assert(!IsSuccessfulAck(0));
    assert(!IsSuccessfulAck(0xff));
    response[2] = 0x01;
    CommandResponseState state = {};
    assert(state.Accept(response, sizeof(response)) == AcceptResult::NotAwaiting);
    assert(!state.completed); // Unsolicited/late replies cannot arm a command.
    state = {true, false, 0x0C, 0x02, 0};
    assert(state.Accept(response, 7) == AcceptResult::Malformed);
    assert(!state.completed);
    response[3] = 0x04;
    assert(state.Accept(response, sizeof(response)) == AcceptResult::Mismatch);
    assert(!state.completed); // Wrong subcommand.
    response[3] = 0x02;
    response[0] = 0x09;
    assert(state.Accept(response, sizeof(response)) == AcceptResult::Mismatch);
    assert(!state.completed); // Wrong command.
    response[0] = 0x0C;
    assert(state.Accept(response, sizeof(response)) == AcceptResult::Accepted); // Fast reply, before write returns.
    assert(state.completed && IsSuccessfulAck(state.ack));
    response[5] = 0xFF;
    assert(state.Accept(response, sizeof(response)) == AcceptResult::Duplicate);
    assert(IsSuccessfulAck(state.ack)); // A second reply must not overwrite it.
    state = {true, false, 0x0C, 0x02, 0};
    assert(state.Accept(response, sizeof(response)) == AcceptResult::Accepted);
    assert(state.completed && !IsSuccessfulAck(state.ack));
    state = {}; // Disconnect cancels the transaction.
    assert(state.Accept(response, sizeof(response)) == AcceptResult::NotAwaiting);
    assert(!state.completed);
    // Every bridge failure cause must have a distinct, named log description.
    for (std::uint32_t e = 1; e <= 9; ++e) {
        const char *name = DescribeResult(MakeResultValue(static_cast<Error>(e)));
        assert(std::strcmp(name, "switch2-unknown") != 0 && std::strcmp(name, "system") != 0);
        for (std::uint32_t other = 1; other < e; ++other) {
            assert(std::strcmp(name, DescribeResult(MakeResultValue(static_cast<Error>(other)))) != 0);
        }
    }
    assert(MakeResultValue(Error::AckTimeout) == ((0x123u & 0x1FF) | (3u << 9)));
    assert(std::strcmp(DescribeResult(0), "success") == 0);
    assert(std::strcmp(DescribeResult(0x0000CA71), "system") == 0);
    for (unsigned int x = 0; x < 4096; ++x) {
        const unsigned int y = 4095 - x;
        const std::uint8_t packed[] = {
            static_cast<std::uint8_t>(x),
            static_cast<std::uint8_t>((x >> 8) | ((y & 15) << 4)),
            static_cast<std::uint8_t>(y >> 4)
        };
        assert(StickX(packed) == x && StickY(packed) == y);
    }
    std::puts("Switch2 protocol regression tests passed");
}
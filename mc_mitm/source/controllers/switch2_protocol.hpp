// SPDX-License-Identifier: GPL-2.0-only
#pragma once
#include <cstddef>
#include <cstdint>

namespace ams::controller::switch2 {

    // Nintendo vendor id and Switch 2 product ids. Source: SDL usb_ids.h, the
    // USB descriptors in documentation/descriptors.md and the advertisement
    // example in documentation/bluetooth_interface.md ("7e 05 69 20" = Pro).
    constexpr std::uint16_t VendorId   = 0x057E;
    constexpr std::uint16_t PidJoyConR = 0x2066;
    constexpr std::uint16_t PidJoyConL = 0x2067;
    constexpr std::uint16_t PidPro     = 0x2069;
    constexpr std::uint16_t PidNsoGc   = 0x2073;

    // BLE omits the report ID. The bridge must restore it from the characteristic
    // UUID, never guess it from payload length (all formats can be 63 bytes).
    constexpr std::uint8_t DefaultReportId(std::uint16_t pid) {
        switch (pid) {
            case PidJoyConL: return 0x07;
            case PidJoyConR: return 0x08;
            case PidPro:     return 0x09;
            case PidNsoGc:   return 0x0A;
            default: return 0;
        }
    }

    constexpr bool IsSwitch2Pid(std::uint16_t pid) {
        return DefaultReportId(pid) != 0;
    }

    constexpr std::size_t MinimumPayloadSize(std::uint8_t report_id) {
        switch (report_id) {
            case 0x05: return 0x10;
            case 0x07:
            case 0x08: return 0x08;
            case 0x09: return 0x0B;
            case 0x0A: return 0x0E;
            default: return 0;
        }
    }

    constexpr bool ValidInput(std::uint8_t report_id, std::uint16_t pid, std::size_t size) {
        const auto minimum = MinimumPayloadSize(report_id);
        return minimum != 0 && size >= minimum && size <= 0x200 &&
               DefaultReportId(pid) != 0 &&
               (report_id == 0x05 || report_id == DefaultReportId(pid));
    }

    constexpr bool IsCommandResponse(const std::uint8_t *data, std::size_t size) {
        // documentation/commands.md: direction=01, transport=01 (Bluetooth).
        return data != nullptr && size >= 8 && size <= 0x200 &&
               data[1] == 0x01 && data[2] == 0x01;
    }

    constexpr bool IsSuccessfulAck(std::uint8_t ack) {
        // Captured normal Bluetooth command responses use 0x78; do not treat
        // direction byte 0x01 as a status or accept arbitrary ACK values.
        return ack == 0x78;
    }

    // Result module/descriptions returned by the Switch 2 bridge. Each failure
    // cause has its own description so the debug log identifies it exactly.
    constexpr std::uint32_t ResultModule = 0x123;
    enum class Error : std::uint32_t {
        StaleConnection     = 1,
        UnsupportedType     = 2,
        AckTimeout          = 3,
        AckRejected         = 4,
        ServiceNotFound     = 5,
        InvalidCommand      = 6,
        DuplicateHandler    = 7,
        GattPathMissing     = 8,
        CommandStateMissing = 9,
    };

    constexpr std::uint32_t MakeResultValue(Error error) {
        // Same encoding as libnx MAKERESULT(module, description).
        return (ResultModule & 0x1FF) | ((static_cast<std::uint32_t>(error) & 0x1FFF) << 9);
    }

    constexpr const char *DescribeResult(std::uint32_t value) {
        if (value == 0) return "success";
        if ((value & 0x1FF) != ResultModule) return "system";
        switch (static_cast<Error>((value >> 9) & 0x1FFF)) {
            case Error::StaleConnection:     return "stale-or-disconnected";
            case Error::UnsupportedType:     return "unsupported-type";
            case Error::AckTimeout:          return "ack-timeout";
            case Error::AckRejected:         return "ack-rejected";
            case Error::ServiceNotFound:     return "service-not-found";
            case Error::InvalidCommand:      return "invalid-command";
            case Error::DuplicateHandler:    return "duplicate-handler";
            case Error::GattPathMissing:     return "gatt-path-missing";
            case Error::CommandStateMissing: return "command-state-missing";
            default:                         return "switch2-unknown";
        }
    }

    enum class AcceptResult : std::uint8_t {
        Accepted,
        Malformed,
        NotAwaiting,
        Duplicate,
        Mismatch,
    };

    constexpr const char *AcceptResultName(AcceptResult result) {
        switch (result) {
            case AcceptResult::Accepted:    return "accepted";
            case AcceptResult::Malformed:   return "malformed";
            case AcceptResult::NotAwaiting: return "unsolicited";
            case AcceptResult::Duplicate:   return "duplicate";
            case AcceptResult::Mismatch:    return "cmd-mismatch";
            default:                        return "unknown";
        }
    }

    // Arm before the write; accept only the first matching complete response.
    // Synchronization and connection lifetime are owned by the BLE bridge.
    struct CommandResponseState {
        bool awaiting = false;
        bool completed = false;
        std::uint8_t command = 0;
        std::uint8_t subcommand = 0;
        std::uint8_t ack = 0;

        AcceptResult Accept(const std::uint8_t *data, std::size_t size) {
            if (!IsCommandResponse(data, size)) return AcceptResult::Malformed;
            if (!awaiting) return AcceptResult::NotAwaiting;
            if (completed) return AcceptResult::Duplicate;
            if (command != data[0] || subcommand != data[3]) return AcceptResult::Mismatch;
            ack = data[5];
            completed = true;
            return AcceptResult::Accepted;
        }
    };

    constexpr std::uint16_t StickX(const std::uint8_t *data) {
        return ((data[1] & 0x0F) << 8) | data[0];
    }

    constexpr std::uint16_t StickY(const std::uint8_t *data) {
        return (data[2] << 4) | (data[1] >> 4);
    }
}
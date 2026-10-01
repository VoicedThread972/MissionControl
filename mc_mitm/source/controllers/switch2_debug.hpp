/*
 * Copyright (c) 2020-2026 ndeadly
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#include <stratosphere.hpp>
#include "../bluetooth_mitm/bluetooth/bluetooth_types.hpp"
#include "switch2_protocol.hpp"

namespace ams::controller {

    // Log levels
    enum class Switch2LogLevel : u8 {
        None    = 0,
        Error   = 1,
        Warning = 2,
        Info    = 3,
        Verbose = 4,
    };

    // Stage markers. Only messages containing one of these markers are written:
    //   [S0] session/startup/config      [S1] BLE scan and raw BLE events
    //   [S2] Switch 2 identification      [S3] connection and GATT setup
    //   [S4] command protocol (TX/RX/ACK) [S5] input notifications and forwarding
    // Result tags: [OK], [FAIL], [DROP], [SKIP], [STATS]. The first [FAIL] or
    // [DROP] after the last [OK] of a connection identifies the failing step.

    // Initialize a fresh file-only session at sdmc:/config/MissionControl/switch2_debug.log.
    // Serialized with logging/finalization; setup failure leaves logging disabled.
    // No GDB output. Session header and subsequent writes are synchronous/best-effort.
    void Switch2DebugInit(Switch2LogLevel level = Switch2LogLevel::Verbose);
    // Disable logging under the logger mutex; file handles are already short-lived.
    void Switch2DebugFini();

    // Log only messages whose format and formatted text contain an [S0]-[S5] marker.
    // State, sequence and static scratch buffers are mutex-protected through file I/O.
    // Long messages are truncated but retain a trailing newline; callers may block on SD I/O.
    void Switch2DebugLog(Switch2LogLevel level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

    // Same as Switch2DebugLog, then appends " len=<size> data=<hex>" (first 64 bytes,
    // "..." if truncated). Hex is formatted in the logger's protected scratch buffer.
    void Switch2DebugLogData(Switch2LogLevel level, const void *data, size_t size, const char *fmt, ...) __attribute__((format(printf, 4, 5)));

    // Convenience macros
    #define SW2_LOG_ERROR(fmt, ...)   ::ams::controller::Switch2DebugLog(::ams::controller::Switch2LogLevel::Error,   "[ERROR] " fmt, ##__VA_ARGS__)
    #define SW2_LOG_WARN(fmt, ...)    ::ams::controller::Switch2DebugLog(::ams::controller::Switch2LogLevel::Warning, "[WARN]  " fmt, ##__VA_ARGS__)
    #define SW2_LOG_INFO(fmt, ...)    ::ams::controller::Switch2DebugLog(::ams::controller::Switch2LogLevel::Info,    "[INFO]  " fmt, ##__VA_ARGS__)
    #define SW2_LOG_VERBOSE(fmt, ...) ::ams::controller::Switch2DebugLog(::ams::controller::Switch2LogLevel::Verbose, "[VERB]  " fmt, ##__VA_ARGS__)

    #define SW2_LOG_DATA_WARN(data, size, fmt, ...)    ::ams::controller::Switch2DebugLogData(::ams::controller::Switch2LogLevel::Warning, data, size, "[WARN]  " fmt, ##__VA_ARGS__)
    #define SW2_LOG_DATA_INFO(data, size, fmt, ...)    ::ams::controller::Switch2DebugLogData(::ams::controller::Switch2LogLevel::Info,    data, size, "[INFO]  " fmt, ##__VA_ARGS__)
    #define SW2_LOG_DATA_VERBOSE(data, size, fmt, ...) ::ams::controller::Switch2DebugLogData(::ams::controller::Switch2LogLevel::Verbose, data, size, "[VERB]  " fmt, ##__VA_ARGS__)

    // Format arguments for a Result: "rc=0x%08X(%s)".
    #define SW2_RC_FMT "rc=0x%08X(%s)"
    #define SW2_RC_ARGS(rc) static_cast<u32>((rc).GetValue()), ::ams::controller::switch2::DescribeResult(static_cast<u32>((rc).GetValue()))

}

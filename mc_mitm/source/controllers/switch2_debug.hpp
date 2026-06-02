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

namespace ams::controller {

    // Log levels
    enum class Switch2LogLevel : u8 {
        None    = 0,
        Error   = 1,
        Warning = 2,
        Info    = 3,
        Verbose = 4,
    };

    // Packet direction
    enum class PacketDirection : u8 {
        In  = 0,
        Out = 1,
    };

    // Initialize the debug logger. Call once at startup.
    // Writes to sdmc:/config/MissionControl/switch2_debug.log
    void Switch2DebugInit(Switch2LogLevel level = Switch2LogLevel::Verbose);
    void Switch2DebugFini();

    // Log a message
    void Switch2DebugLog(Switch2LogLevel level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

    // Log a raw HID report (hex dump)
    void Switch2DebugLogReport(PacketDirection dir, const bluetooth::Address &addr,
                               const u8 *data, size_t size);

    // Log a Bluetooth event (connection, disconnection, etc.)
    void Switch2DebugLogBtEvent(const char *event, const bluetooth::Address &addr);

    // Convenience macros
    #define SW2_LOG_ERROR(fmt, ...)   ::ams::controller::Switch2DebugLog(::ams::controller::Switch2LogLevel::Error,   "[ERROR] " fmt, ##__VA_ARGS__)
    #define SW2_LOG_WARN(fmt, ...)    ::ams::controller::Switch2DebugLog(::ams::controller::Switch2LogLevel::Warning, "[WARN]  " fmt, ##__VA_ARGS__)
    #define SW2_LOG_INFO(fmt, ...)    ::ams::controller::Switch2DebugLog(::ams::controller::Switch2LogLevel::Info,    "[INFO]  " fmt, ##__VA_ARGS__)
    #define SW2_LOG_VERBOSE(fmt, ...) ::ams::controller::Switch2DebugLog(::ams::controller::Switch2LogLevel::Verbose, "[VERB]  " fmt, ##__VA_ARGS__)

}

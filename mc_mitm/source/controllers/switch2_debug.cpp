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
#include "switch2_debug.hpp"
#include <cstdio>
#include <cstring>
#include <cstdarg>

namespace ams::controller {

    namespace {

        constexpr const char *LogPath = "sdmc:/config/MissionControl/switch2_debug.log";
        constexpr size_t      LogBufSize = 4096;

        os::SdkMutex  g_log_mutex;
        fs::FileHandle g_log_file;
        bool           g_log_open    = false;
        Switch2LogLevel g_log_level  = Switch2LogLevel::Verbose;

        // Write raw bytes to the open log file, ignoring errors
        void WriteRaw(const char *buf, size_t len) {
            if (!g_log_open || len == 0) return;
            s64 offset = 0;
            // Seek to end
            if (R_FAILED(fs::GetFileSize(std::addressof(offset), g_log_file))) return;
            // Extend file if needed
            fs::SetFileSize(g_log_file, offset + static_cast<s64>(len));
            fs::WriteFile(g_log_file, offset, buf, len, fs::WriteOption::None);
            fs::FlushFile(g_log_file);
        }

        void FormatAddress(char *buf, size_t bufsz, const bluetooth::Address &addr) {
            std::snprintf(buf, bufsz, "%02X:%02X:%02X:%02X:%02X:%02X",
                addr.address[0], addr.address[1], addr.address[2],
                addr.address[3], addr.address[4], addr.address[5]);
        }

    }

    void Switch2DebugInit(Switch2LogLevel level) {
        std::scoped_lock lk(g_log_mutex);
        g_log_level = level;

        // Ensure directory exists
        fs::EnsureDirectory("sdmc:/config/MissionControl");

        // Delete old log and create fresh
        fs::DeleteFile(LogPath);
        if (R_FAILED(fs::CreateFile(LogPath, 0))) return;
        if (R_FAILED(fs::OpenFile(std::addressof(g_log_file), LogPath, fs::OpenMode_ReadWrite | fs::OpenMode_AllowAppend))) return;
        g_log_open = true;

        const char *header = "=== Switch2 Debug Log ===\n";
        WriteRaw(header, std::strlen(header));
    }

    void Switch2DebugFini() {
        std::scoped_lock lk(g_log_mutex);
        if (g_log_open) {
            fs::CloseFile(g_log_file);
            g_log_open = false;
        }
    }

    void Switch2DebugLog(Switch2LogLevel level, const char *fmt, ...) {
        if (level > g_log_level) return;

        char buf[512];
        u64 tick = os::GetSystemTick().GetInt64Value();

        // Format the user message
        char msg[400];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(msg, sizeof(msg), fmt, args);
        va_end(args);

        int len = std::snprintf(buf, sizeof(buf), "[%016llX] %s\n", (unsigned long long)tick, msg);
        if (len <= 0) return;

        std::scoped_lock lk(g_log_mutex);
        WriteRaw(buf, static_cast<size_t>(len));
    }

    void Switch2DebugLogReport(PacketDirection dir, const bluetooth::Address &addr,
                               const u8 *data, size_t size) {
        if (Switch2LogLevel::Verbose > g_log_level) return;

        char addrStr[20];
        FormatAddress(addrStr, sizeof(addrStr), addr);

        u64 tick = os::GetSystemTick().GetInt64Value();

        // Build hex string (cap at 64 bytes to keep log manageable)
        char hexbuf[256];
        size_t printSize = size > 64 ? 64 : size;
        size_t pos = 0;
        for (size_t i = 0; i < printSize && pos + 3 < sizeof(hexbuf); ++i) {
            pos += std::snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "%02X ", data[i]);
        }
        if (size > 64) {
            std::snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "...(+%zu)", size - 64);
        }

        char line[512];
        int len = std::snprintf(line, sizeof(line), "[%016llX] [%s %s] %s\n",
            (unsigned long long)tick,
            dir == PacketDirection::In ? "IN " : "OUT",
            addrStr, hexbuf);
        if (len <= 0) return;

        std::scoped_lock lk(g_log_mutex);
        WriteRaw(line, static_cast<size_t>(len));
    }

    void Switch2DebugLogBtEvent(const char *event, const bluetooth::Address &addr) {
        char addrStr[20];
        FormatAddress(addrStr, sizeof(addrStr), addr);
        SW2_LOG_INFO("BT Event: %s addr=%s", event, addrStr);
    }

}

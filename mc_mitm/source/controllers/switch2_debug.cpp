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
#include <algorithm>

namespace ams::controller {

    namespace {

        constexpr const char *LogPath = "sdmc:/config/MissionControl/switch2_debug.log";
        constexpr const char *BuildSignature = __DATE__ " " __TIME__;

        bool IsAllowedDiagnosticMessage(const char *msg) {
            if (msg == nullptr) {
                return false;
            }

            return std::strstr(msg, "[S0]") != nullptr ||
                   std::strstr(msg, "[S1]") != nullptr ||
                   std::strstr(msg, "[S2]") != nullptr ||
                   std::strstr(msg, "[S3]") != nullptr ||
                   std::strstr(msg, "[S4]") != nullptr ||
                   std::strstr(msg, "[S5]") != nullptr ||
                   std::strstr(msg, "[S6]") != nullptr;
        }

        // All mutable logger state and formatting buffers are protected by g_log_mutex.
        os::SdkMutex   g_log_mutex;
        bool           g_initialized = false;
        Switch2LogLevel g_log_level  = Switch2LogLevel::Verbose;
        u64            g_log_sequence = 0;
        char           g_message[448];
        char           g_line[512];
        char           g_tick_line[24];

        Result EnsureLogFileExists() {
            R_TRY_CATCH(fs::CreateFile(LogPath, 0)) {
                R_CONVERT(fs::ResultPathAlreadyExists, ResultSuccess());
            } R_END_TRY_CATCH;

            R_SUCCEED();
        }

        // Caller must hold g_log_mutex. File-only, synchronous, best-effort writes;
        // short-lived handles allow SD tools access between writes, not coordination.
        void WriteRaw(const char *buf, size_t len) {
            if (!g_initialized || len == 0) return;

            if (R_FAILED(EnsureLogFileExists())) return;

            fs::FileHandle file;
            if (R_FAILED(fs::OpenFile(std::addressof(file), LogPath, fs::OpenMode_Write | fs::OpenMode_AllowAppend))) return;
            ON_SCOPE_EXIT { fs::CloseFile(file); };

            s64 offset = 0;
            if (R_FAILED(fs::GetFileSize(std::addressof(offset), file))) return;

            static_cast<void>(fs::WriteFile(file, offset, buf, len, fs::WriteOption::Flush));
        }

        void WriteSessionHeader() {
            // Caller must hold g_log_mutex; shared scratch storage keeps tiny stacks small.
            static constexpr const char kHeaderPrefix[] =
                "=== Switch2 Debug Log ===\n"
                "Build Signature: ";
            static constexpr const char kHeaderMid[] =
                "\nSession Start Tick: ";
            static constexpr const char kHeaderSuffix[] =
                "\nLog Format Version: 4\n"
                "Stages: S0=session S1=ble-scan/events S2=identify S3=connect/gatt S4=commands S5=input S6=horizon-hdls\n";

            WriteRaw(kHeaderPrefix, std::strlen(kHeaderPrefix));
            WriteRaw(BuildSignature, std::strlen(BuildSignature));
            WriteRaw(kHeaderMid, std::strlen(kHeaderMid));

            const u64 session_tick = os::GetSystemTick().GetInt64Value();
            const int tick_len = std::snprintf(g_tick_line, sizeof(g_tick_line), "%016llX", static_cast<unsigned long long>(session_tick));
            if (tick_len > 0) {
                const size_t written = static_cast<size_t>(tick_len) < sizeof(g_tick_line)
                    ? static_cast<size_t>(tick_len) : sizeof(g_tick_line) - 1;
                WriteRaw(g_tick_line, written);
            }

            WriteRaw(kHeaderSuffix, std::strlen(kHeaderSuffix));
        }

    }

    void Switch2DebugInit(Switch2LogLevel level) {
        std::scoped_lock lk(g_log_mutex);
        // A failed reinitialization must not leave the previous session enabled.
        g_initialized = false;
        g_log_level = level;
        g_log_sequence = 0;

        // Leave logging disabled on any setup failure; a missing old log is harmless.
        if (R_FAILED(fs::EnsureDirectory("sdmc:/config/MissionControl"))) return;

        const Result delete_result = fs::DeleteFile(LogPath);
        if (R_FAILED(delete_result) && !fs::ResultPathNotFound::Includes(delete_result)) return;
        if (R_FAILED(fs::CreateFile(LogPath, 0))) return;
        g_initialized = true;

        WriteSessionHeader();
    }

    void Switch2DebugFini() {
        std::scoped_lock lk(g_log_mutex);
        g_initialized = false;
    }

    namespace {

        // Caller must hold g_log_mutex. Appends " len=N data=HEX" to g_message.
        size_t AppendHexLocked(size_t used, const u8 *data, size_t size) {
            constexpr size_t MaxHexBytes = 64;
            static constexpr char Digits[] = "0123456789ABCDEF";

            const int n = std::snprintf(g_message + used, sizeof(g_message) - used, " len=%u data=", static_cast<unsigned int>(size));
            if (n < 0) return used;
            used = std::min(used + static_cast<size_t>(n), sizeof(g_message) - 1);

            const size_t shown = data != nullptr ? std::min(size, MaxHexBytes) : 0;
            for (size_t i = 0; i < shown && used + 2 < sizeof(g_message); ++i) {
                g_message[used++] = Digits[data[i] >> 4];
                g_message[used++] = Digits[data[i] & 0xF];
            }
            if (size > shown && used + 3 < sizeof(g_message)) {
                g_message[used++] = '.';
                g_message[used++] = '.';
                g_message[used++] = '.';
            }
            g_message[used] = '\0';
            return used;
        }

        void LogImpl(Switch2LogLevel level, bool has_data, const u8 *data, size_t size, const char *fmt, va_list args) {
            if (fmt == nullptr) return;

            // This fast path inspects only the caller's format, never shared logger state.
            if (!IsAllowedDiagnosticMessage(fmt)) {
                return;
            }

            // Serialize state checks, formatting, sequence assignment and synchronous I/O.
            std::scoped_lock lk(g_log_mutex);
            if (!g_initialized || level > g_log_level) return;

            // Use shared scratch buffers instead of consuming the caller's tiny stack.
            const int message_len = std::vsnprintf(g_message, sizeof(g_message), fmt, args);
            if (message_len < 0) return;

            size_t used = std::min(static_cast<size_t>(message_len), sizeof(g_message) - 1);
            if (has_data) {
                used = AppendHexLocked(used, data, size);
            }

            // Keep the runtime log output focused on the requested diagnosis channels.
            if (!IsAllowedDiagnosticMessage(g_message)) {
                return;
            }

            const u64 tick = os::GetSystemTick().GetInt64Value();
            ++g_log_sequence;
            const int len = std::snprintf(
                g_line,
                sizeof(g_line),
                "[#%06llu][%016llX] %s\n",
                static_cast<unsigned long long>(g_log_sequence),
                static_cast<unsigned long long>(tick),
                g_message
            );
            if (len <= 0) return;

            // snprintf returns the required length, not the stored length. Never write
            // beyond the buffer or include its NUL; retain a newline even if truncated.
            const size_t written = static_cast<size_t>(len) < sizeof(g_line)
                ? static_cast<size_t>(len) : sizeof(g_line) - 1;
            g_line[written - 1] = '\n';
            WriteRaw(g_line, written);
        }

    }

    void Switch2DebugLog(Switch2LogLevel level, const char *fmt, ...) {
        va_list args;
        va_start(args, fmt);
        LogImpl(level, false, nullptr, 0, fmt, args);
        va_end(args);
    }

    void Switch2DebugLogData(Switch2LogLevel level, const void *data, size_t size, const char *fmt, ...) {
        va_list args;
        va_start(args, fmt);
        LogImpl(level, true, static_cast<const u8 *>(data), size, fmt, args);
        va_end(args);
    }

}

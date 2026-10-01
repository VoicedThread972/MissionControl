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
#include "bluetooth_ble.hpp"
#include "../btdrv_mitm_flags.hpp"
#include "../../controllers/switch2_debug.hpp"
#include "../../controllers/switch2_discovery.hpp"
#include "../../controllers/controller_management.hpp"
#include "../../controllers/switch2_protocol.hpp"
#include "../../mcmitm_config.hpp"
#include <cstdio>
#include <map>

namespace ams::bluetooth::ble {

    namespace {

        constinit os::SdkMutex g_event_data_lock;
        constinit bluetooth::BleEventInfo g_event_info;
        constinit bluetooth::BleEventType g_current_event_type;

        os::SystemEvent g_system_event;
        os::SystemEvent g_system_event_fwd(os::EventClearMode_AutoClear, true);
        os::SystemEvent g_system_event_user_fwd(os::EventClearMode_AutoClear, true);

        os::Event g_init_event(os::EventClearMode_ManualClear);
        os::Event g_data_read_event(os::EventClearMode_AutoClear);

        constinit os::SdkMutex g_conn_map_lock;
        struct AddressCompare {
            bool operator()(const bluetooth::Address &lhs, const bluetooth::Address &rhs) const {
                for (int i = 0; i < 6; ++i) {
                    if (lhs.address[i] < rhs.address[i]) return true;
                    if (lhs.address[i] > rhs.address[i]) return false;
                }
                return false;
            }
        };
        std::map<u32, bluetooth::Address> g_conn_map;
        std::map<bluetooth::Address, u32, AddressCompare> g_ble_conn_ids;
        std::map<u32, u64> g_connection_generations;
        u64 g_next_connection_generation = 0;

        namespace sw2 = controller::switch2;

        constexpr u32 Sw2Error(sw2::Error error) {
            return sw2::MakeResultValue(error);
        }

        // Reasons a notification on a tracked Switch 2 connection did not reach a handler.
        enum NotifyDrop : u8 {
            NotifyDrop_ForeignService,
            NotifyDrop_UnexpectedChar,
            NotifyDrop_NotReady,
            NotifyDrop_NoHandler,
            NotifyDrop_TooShort,
            NotifyDrop_HandlerError,
            NotifyDrop_Count,
        };

        constexpr const char *NotifyDropNames[NotifyDrop_Count] = {
            "FOREIGN-SERVICE",
            "UNEXPECTED-CHAR",
            "NOT-READY",
            "NO-HANDLER",
            "TOO-SHORT",
            "HANDLER-ERROR",
        };

        // Per-connection counters, guarded by g_conn_map_lock. Logged on disconnect
        // so a session shows how far traffic progressed even without per-packet logs.
        struct Switch2ConnectionStats {
            u32 notifications;
            u32 responses;
            u32 responses_rejected;
            u32 delivered;
            u32 commands_ok;
            u32 commands_failed;
            u32 drops[NotifyDrop_Count];
            u32 logged_drop_mask;
            bool first_input_logged;
        };
        std::map<u32, Switch2ConnectionStats> g_connection_stats;

        constexpr u32 MaxRejectedResponseLogs = 16;
        constexpr u32 InputAliveLogInterval = 1000;

        // Event-thread-only counters for rate-limited scan/notify diagnostics.
        u32 g_scan_results_seen = 0;
        u32 g_nonmatching_0553_logs = 0;
        u32 g_untracked_notify_logs = 0;
        constexpr u32 MaxScanResultLogs = 32;
        constexpr u32 ScanSummaryInterval = 256;
        constexpr u32 MaxNonmatching0553Logs = 16;
        constexpr u32 MaxUntrackedNotifyLogs = 4;

        struct PendingSwitch2ConnectRequest {
            bluetooth::Address address;
            controller::ControllerType type;
        };

        using Switch2CommandResponseState = controller::switch2::CommandResponseState;

        struct PendingSwitch2InitRequest {
            bluetooth::Address address;
            u32 conn_id;
            controller::ControllerType type;
            u64 generation;
        };

        struct Switch2GattPath {
            BtdrvGattId service;
            BtdrvGattId command;
            bool primary;
            controller::ControllerType type;
            bool ready;
        };

        std::map<u32, Switch2GattPath> g_switch2_gatt_paths;
        std::map<u32, PendingSwitch2InitRequest> g_pending_switch2_init_requests;
        constinit os::SdkMutex g_switch2_write_lock;

        Result InitializeSwitch2Gatt(const PendingSwitch2InitRequest &request);

        constinit os::SdkMutex g_switch2_connect_queue_lock;
        os::Event g_switch2_connect_queue_event(os::EventClearMode_AutoClear);
        std::map<bluetooth::Address, PendingSwitch2ConnectRequest, AddressCompare> g_pending_switch2_connect_requests;
        std::map<bluetooth::Address, u64, AddressCompare> g_last_switch2_connect_attempt_ns;

        struct Switch2DiscoveredEntry {
            u32 index;
            controller::ControllerType type;
        };

        constinit os::SdkMutex g_switch2_discovered_list_lock;
        std::map<bluetooth::Address, Switch2DiscoveredEntry, AddressCompare> g_switch2_discovered_list;
        u32 g_switch2_discovered_count = 0;

        constinit os::SdkMutex g_switch2_cmd_response_lock;
        std::map<u32, Switch2CommandResponseState> g_switch2_cmd_responses;

        constexpr s32 Switch2ConnectWorkerThreadPriority = 18;
        constexpr size_t Switch2ConnectWorkerThreadStackSize = 0x3000;
        alignas(os::ThreadStackAlignment) constinit u8 g_switch2_connect_worker_thread_stack[Switch2ConnectWorkerThreadStackSize];
        constinit os::ThreadType g_switch2_connect_worker_thread;
        constinit bool g_switch2_connect_worker_started = false;

        constexpr u64 Switch2ConnectRetryIntervalNs = 2'000'000'000ull;

        constexpr BtdrvGattAttributeUuid Switch2InputServiceUuid = { 16, { 0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad, 0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd0 } };
        constexpr BtdrvGattAttributeUuid Switch2JoyConLInputCharUuid = { 16, { 0xcc, 0x1b, 0xbb, 0xb5, 0x73, 0x54, 0x4d, 0x32, 0xa7, 0x16, 0xa8, 0x1c, 0xb2, 0x41, 0xa3, 0x2a } };
        constexpr BtdrvGattAttributeUuid Switch2JoyConRInputCharUuid = { 16, { 0xd5, 0xa9, 0xe0, 0x1e, 0x2f, 0xfc, 0x4c, 0xca, 0xb2, 0x0c, 0x8b, 0x67, 0x14, 0x2b, 0xf4, 0x42 } };
        constexpr BtdrvGattAttributeUuid Switch2ProInputCharUuid = { 16, { 0x74, 0x92, 0x86, 0x6c, 0xec, 0x3e, 0x46, 0x19, 0x82, 0x58, 0x32, 0x75, 0x5f, 0xfc, 0xc0, 0xf8 } };
        constexpr BtdrvGattAttributeUuid Switch2NsoGcInputCharUuid = { 16, { 0x82, 0x61, 0xcb, 0xa1, 0x94, 0x35, 0x42, 0x0c, 0x84, 0xd6, 0xf0, 0xc7, 0x5a, 0x2c, 0x8e, 0x4d } };
        constexpr BtdrvGattAttributeUuid Switch2CommandCharUuid = { 16, { 0x64, 0x9d, 0x4a, 0xc9, 0x8e, 0xb7, 0x4e, 0x6c, 0xaf, 0x44, 0x1e, 0xa5, 0x4f, 0xe5, 0xf0, 0x05 } };
        constexpr BtdrvGattAttributeUuid Switch2CommandResponse1CharUuid = { 16, { 0xc7, 0x65, 0xa9, 0x61, 0xd9, 0xd8, 0x4d, 0x36, 0xa2, 0x0a, 0x53, 0x15, 0xb1, 0x11, 0x83, 0x6a } };

        [[maybe_unused]] const char *GetBleEventTypeName(const bluetooth::BleEventType type) {
            switch (type) {
                case BtdrvBleEventType_ClientRegistration:            return "ClientRegistration";
                case BtdrvBleEventType_ServerRegistration:            return "ServerRegistration";
                case BtdrvBleEventType_ConnectionUpdate:              return "ConnectionUpdate";
                case BtdrvBleEventType_PreferredConnectionParameters: return "PreferredConnectionParameters";
                case BtdrvBleEventType_ClientConnection:              return "ClientConnection";
                case BtdrvBleEventType_ServerConnection:              return "ServerConnection";
                case BtdrvBleEventType_ScanResult:                    return "ScanResult";
                case BtdrvBleEventType_ScanFilter:                    return "ScanFilter";
                case BtdrvBleEventType_ClientNotify:                  return "ClientNotify";
                case BtdrvBleEventType_ClientCacheSave:               return "ClientCacheSave";
                case BtdrvBleEventType_ClientCacheLoad:               return "ClientCacheLoad";
                case BtdrvBleEventType_ClientConfigureMtu:            return "ClientConfigureMtu";
                case BtdrvBleEventType_ServerAddAttribute:            return "ServerAddAttribute";
                case BtdrvBleEventType_ServerAttributeOperation:      return "ServerAttributeOperation";
                default:                                              return "Unknown";
            }
        }

        const char *GetScanStatusName(const u8 status) {
            switch (status) {
                case 0xFF: return "ScanStarted";
                case 0x01: return "ScanComplete";
                case 0x02: return "DeviceFound";
                default:   return "Unknown";
            }
        }

        [[maybe_unused]] void FormatGattUuid(char *buf, size_t bufsz, const BtdrvGattAttributeUuid &uuid) {
            if (uuid.size == 16) {
                std::snprintf(
                    buf,
                    bufsz,
                    "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                    uuid.uuid[0], uuid.uuid[1], uuid.uuid[2], uuid.uuid[3],
                    uuid.uuid[4], uuid.uuid[5],
                    uuid.uuid[6], uuid.uuid[7],
                    uuid.uuid[8], uuid.uuid[9],
                    uuid.uuid[10], uuid.uuid[11], uuid.uuid[12], uuid.uuid[13], uuid.uuid[14], uuid.uuid[15]
                );
            } else {
                std::snprintf(buf, bufsz, "size=%u", uuid.size);
            }
        }

        BtdrvGattId MakeGattId(const BtdrvGattAttributeUuid &uuid) {
            return BtdrvGattId{ 0, { 0, 0, 0 }, uuid };
        }

        bool UuidEquals(const BtdrvGattAttributeUuid &lhs, const BtdrvGattAttributeUuid &rhs) {
            return (lhs.size == 2 || lhs.size == 4 || lhs.size == 16) &&
                   lhs.size == rhs.size && std::memcmp(lhs.uuid, rhs.uuid, lhs.size) == 0;
        }

        const BtdrvGattAttributeUuid *GetSwitch2InputCharUuid(controller::ControllerType type) {
            switch (type) {
                case controller::ControllerType_Switch2JoyConL:        return &Switch2JoyConLInputCharUuid;
                case controller::ControllerType_Switch2JoyConR:        return &Switch2JoyConRInputCharUuid;
                case controller::ControllerType_Switch2ProController:  return &Switch2ProInputCharUuid;
                case controller::ControllerType_Switch2NSOGCController:return &Switch2NsoGcInputCharUuid;
                default:                                               return nullptr;
            }
        }


        bool IsSwitch2ControllerType(controller::ControllerType type) {
            switch (type) {
                case controller::ControllerType_Switch2JoyConL:
                case controller::ControllerType_Switch2JoyConR:
                case controller::ControllerType_Switch2ProController:
                case controller::ControllerType_Switch2NSOGCController:
                    return true;
                default:
                    return false;
            }
        }

        const char *GetSwitch2ControllerTypeName(controller::ControllerType type) {
            switch (type) {
                case controller::ControllerType_Switch2JoyConL:         return "Joy-Con 2 L";
                case controller::ControllerType_Switch2JoyConR:         return "Joy-Con 2 R";
                case controller::ControllerType_Switch2ProController:   return "Pro Controller 2";
                case controller::ControllerType_Switch2NSOGCController: return "NSO GC Controller 2";
                default:                                                return "Unknown";
            }
        }

        controller::ControllerType ResolveSwitch2TypeFromPairedInfo(const bluetooth::Address &address) {
            bluetooth::DevicesSettings device_settings = {};
            if (R_FAILED(btdrvGetPairedDeviceInfo(address, &device_settings))) {
                return controller::ControllerType_Unknown;
            }

            const controller::ControllerType type = controller::Identify(&device_settings);
            return IsSwitch2ControllerType(type) ? type : controller::ControllerType_Unknown;
        }

        void RegisterSwitch2DiscoveryListEntry(const bluetooth::Address &address, controller::ControllerType type, u32 *out_index, bool *out_is_new) {
            bool is_new = false;
            u32 index = 0;

            {
                std::scoped_lock lk(g_switch2_discovered_list_lock);
                auto it = g_switch2_discovered_list.find(address);
                if (it == g_switch2_discovered_list.end()) {
                    if (g_switch2_discovered_list.size() >= 32) return;
                    index = ++g_switch2_discovered_count;
                    g_switch2_discovered_list.emplace(address, Switch2DiscoveredEntry{index, type});
                    is_new = true;
                } else {
                    it->second.type = type;
                    index = it->second.index;
                }
            }

            if (out_index) {
                *out_index = index;
            }
            if (out_is_new) {
                *out_is_new = is_new;
            }
        }

        bool HasBleConnection(const bluetooth::Address &address, u32 *out_conn_id) {
            std::scoped_lock lk(g_conn_map_lock);
            auto it = g_ble_conn_ids.find(address);
            if (it == g_ble_conn_ids.end()) {
                return false;
            }

            if (out_conn_id) {
                *out_conn_id = it->second;
            }
            return true;
        }

        bool IsCurrentConnection(const PendingSwitch2InitRequest &request) {
            std::scoped_lock lk(g_conn_map_lock);
            const auto address = g_ble_conn_ids.find(request.address);
            const auto generation = g_connection_generations.find(request.conn_id);
            return address != g_ble_conn_ids.end() && address->second == request.conn_id &&
                   generation != g_connection_generations.end() && generation->second == request.generation;
        }

        // Returns true only for the first drop of this reason on this connection.
        bool CountNotifyDrop(u32 conn_id, NotifyDrop reason) {
            std::scoped_lock lk(g_conn_map_lock);
            const auto it = g_connection_stats.find(conn_id);
            if (it == g_connection_stats.end()) return false;
            ++it->second.drops[reason];
            const u32 bit = 1u << reason;
            if ((it->second.logged_drop_mask & bit) != 0) return false;
            it->second.logged_drop_mask |= bit;
            return true;
        }

        void CountCommand(u32 conn_id, bool ok) {
            std::scoped_lock lk(g_conn_map_lock);
            const auto it = g_connection_stats.find(conn_id);
            if (it == g_connection_stats.end()) return;
            ++(ok ? it->second.commands_ok : it->second.commands_failed);
        }

        void LogConnectionStats(u32 conn_id, const Switch2ConnectionStats &s) {
            SW2_LOG_INFO(
                "[S5][STATS] conn_id=%u notify=%u responses=%u rejected=%u delivered=%u cmd_ok=%u cmd_fail=%u "
                "drop{foreign=%u unexpected=%u not_ready=%u no_handler=%u short=%u handler=%u}",
                conn_id, s.notifications, s.responses, s.responses_rejected, s.delivered, s.commands_ok, s.commands_failed,
                s.drops[NotifyDrop_ForeignService], s.drops[NotifyDrop_UnexpectedChar], s.drops[NotifyDrop_NotReady],
                s.drops[NotifyDrop_NoHandler], s.drops[NotifyDrop_TooShort], s.drops[NotifyDrop_HandlerError]
            );
        }

        Result LogGattStep(u32 conn_id, const char *target, const char *step, Result rc) {
            if (R_FAILED(rc)) {
                SW2_LOG_WARN("[S3][FAIL][GATT-%s-%s] conn_id=%u " SW2_RC_FMT, target, step, conn_id, SW2_RC_ARGS(rc));
            } else {
                SW2_LOG_INFO("[S3][OK][GATT-%s-%s] conn_id=%u", target, step, conn_id);
            }
            return rc;
        }

        Result LogStaleConnection(const PendingSwitch2InitRequest &request, const char *step) {
            SW2_LOG_WARN("[S3][FAIL][GATT-STALE] conn_id=%u generation=%llu step=%s connection replaced or disconnected",
                request.conn_id, static_cast<unsigned long long>(request.generation), step);
            return Sw2Error(sw2::Error::StaleConnection);
        }

        void QueueSwitch2Initialize(const PendingSwitch2InitRequest &request) {
            std::scoped_lock lk(g_switch2_connect_queue_lock);
            // btm supports at most four concurrent BLE connections.
            if (g_pending_switch2_init_requests.size() >= 4) return;
            g_pending_switch2_init_requests.emplace(request.conn_id, request);
            g_switch2_connect_queue_event.Signal();
        }

        bool PopSwitch2Initialize(PendingSwitch2InitRequest *request) {
            std::scoped_lock lk(g_switch2_connect_queue_lock);
            if (g_pending_switch2_init_requests.empty()) return false;
            auto it = g_pending_switch2_init_requests.begin();
            *request = it->second;
            g_pending_switch2_init_requests.erase(it);
            return true;
        }

        u64 GetCurrentTimeNs() {
            return armTicksToNs(armGetSystemTick());
        }

        Result ConnectSwitch2Controller(const bluetooth::Address &address) {
            R_TRY(btmInitialize());
            ON_SCOPE_EXIT { btmExit(); };

            R_RETURN(btmBleConnect(address));
        }

        bool PopPendingSwitch2Connect(PendingSwitch2ConnectRequest *out_request) {
            std::scoped_lock lk(g_switch2_connect_queue_lock);
            if (g_pending_switch2_connect_requests.empty()) {
                return false;
            }

            auto it = g_pending_switch2_connect_requests.begin();
            *out_request = it->second;
            g_pending_switch2_connect_requests.erase(it);
            g_last_switch2_connect_attempt_ns[out_request->address] = GetCurrentTimeNs();
            return true;
        }

        void QueueSwitch2ConnectRequest(const bluetooth::Address &address, controller::ControllerType type, const char *reason) {
            if (!IsSwitch2ControllerType(type)) {
                return;
            }

            if (HasBleConnection(address, nullptr)) {
                return;
            }

            bool inserted = false;
            bool queue_full = false;
            {
                std::scoped_lock lk(g_switch2_connect_queue_lock);

                const u64 now_ns = GetCurrentTimeNs();
                if (g_pending_switch2_connect_requests.size() >= 4 ||
                    (g_last_switch2_connect_attempt_ns.size() >= 32 &&
                     g_last_switch2_connect_attempt_ns.find(address) == g_last_switch2_connect_attempt_ns.end())) {
                    queue_full = true;
                } else {
                    auto last_attempt_it = g_last_switch2_connect_attempt_ns.find(address);
                    if (last_attempt_it != g_last_switch2_connect_attempt_ns.end()) {
                        if (now_ns - last_attempt_it->second < Switch2ConnectRetryIntervalNs) {
                            return;
                        }
                    }

                    auto [it, was_inserted] = g_pending_switch2_connect_requests.emplace(address, PendingSwitch2ConnectRequest{address, type});
                    if (!was_inserted) {
                        it->second.type = type;
                    } else {
                        inserted = true;
                    }
                }
            }

            if (queue_full) {
                SW2_LOG_WARN(
                    "[S3][FAIL][CONNECT-QUEUE] queue/table full type=%s mac=%02X:%02X:%02X:%02X:%02X:%02X",
                    GetSwitch2ControllerTypeName(type),
                    address.address[0], address.address[1], address.address[2],
                    address.address[3], address.address[4], address.address[5]
                );
                return;
            }

            if (!inserted) {
                return;
            }

            SW2_LOG_INFO(
                "[S3][OK][CONNECT-QUEUE] reason=%s type=%s mac=%02X:%02X:%02X:%02X:%02X:%02X",
                reason,
                GetSwitch2ControllerTypeName(type),
                address.address[0], address.address[1], address.address[2],
                address.address[3], address.address[4], address.address[5]
            );

            g_switch2_connect_queue_event.Signal();
        }

        void Switch2ConnectWorkerThreadFunc(void *) {
            for (;;) {
                g_switch2_connect_queue_event.Wait();

                PendingSwitch2ConnectRequest request = {};
                while (PopPendingSwitch2Connect(&request)) {
                    if (HasBleConnection(request.address, nullptr)) {
                        continue;
                    }

                    SW2_LOG_INFO(
                        "[S3][CONNECT-ATTEMPT] type=%s mac=%02X:%02X:%02X:%02X:%02X:%02X",
                        GetSwitch2ControllerTypeName(request.type),
                        request.address.address[0], request.address.address[1], request.address.address[2],
                        request.address.address[3], request.address.address[4], request.address.address[5]
                    );

                    const Result rc_connect = ConnectSwitch2Controller(request.address);
                    if (R_FAILED(rc_connect)) {
                        SW2_LOG_WARN("[S3][FAIL][CONNECT-REQUEST] " SW2_RC_FMT, SW2_RC_ARGS(rc_connect));
                    } else {
                        SW2_LOG_INFO("[S3][OK][CONNECT-REQUEST] submitted; expect [S3][OK][CONNECTED] next");
                    }

                    os::SleepThread(ams::TimeSpan::FromMilliSeconds(100));
                }

                PendingSwitch2InitRequest init_request = {};
                while (PopSwitch2Initialize(&init_request)) {
                    const u64 start_ns = GetCurrentTimeNs();
                    const Result rc = InitializeSwitch2Gatt(init_request);
                    const u32 elapsed_ms = static_cast<u32>((GetCurrentTimeNs() - start_ns) / 1'000'000ull);
                    if (R_SUCCEEDED(rc)) {
                        SW2_LOG_INFO("[S3][OK][GATT-SETUP] conn_id=%u elapsed_ms=%u input routing enabled", init_request.conn_id, elapsed_ms);
                    } else {
                        SW2_LOG_WARN("[S3][FAIL][GATT-SETUP] conn_id=%u elapsed_ms=%u " SW2_RC_FMT " (see preceding [FAIL] for the exact step)",
                            init_request.conn_id, elapsed_ms, SW2_RC_ARGS(rc));
                        if (IsCurrentConnection(init_request)) {
                            controller::RemoveHandler(init_request.address);
                            {
                                std::scoped_lock lk(g_conn_map_lock);
                                g_switch2_gatt_paths.erase(init_request.conn_id);
                            }
                            Result rc_disconnect = btmInitialize();
                            if (R_SUCCEEDED(rc_disconnect)) {
                                rc_disconnect = btmBleDisconnect(init_request.conn_id);
                                btmExit();
                            }
                            SW2_LOG_INFO("[S3][CLEANUP] conn_id=%u handler removed, disconnect " SW2_RC_FMT,
                                init_request.conn_id, SW2_RC_ARGS(rc_disconnect));
                        }
                    }
                }
            }
        }

        void StartSwitch2ConnectWorkerIfNeeded() {
            if (g_switch2_connect_worker_started) {
                return;
            }

            R_ABORT_UNLESS(os::CreateThread(
                &g_switch2_connect_worker_thread,
                Switch2ConnectWorkerThreadFunc,
                nullptr,
                g_switch2_connect_worker_thread_stack,
                Switch2ConnectWorkerThreadStackSize,
                Switch2ConnectWorkerThreadPriority
            ));

            os::SetThreadNamePointer(&g_switch2_connect_worker_thread, "mc::Sw2ConnectWorker");
            os::StartThread(&g_switch2_connect_worker_thread);
            g_switch2_connect_worker_started = true;
        }

        void ResetSwitch2CommandState(u32 conn_id) {
            std::scoped_lock lk(g_switch2_cmd_response_lock);
            g_switch2_cmd_responses[conn_id] = {};
        }

        void ClearSwitch2CommandState(u32 conn_id) {
            std::scoped_lock lk(g_switch2_cmd_response_lock);
            g_switch2_cmd_responses.erase(conn_id);
        }

        void RecordSwitch2CommandResponse(u32 conn_id, const u8 *data, u16 size) {
            sw2::AcceptResult result = sw2::AcceptResult::NotAwaiting;
            u8 expected_cmd = 0;
            u8 expected_sub = 0;
            {
                std::scoped_lock lk(g_switch2_cmd_response_lock);
                const auto it = g_switch2_cmd_responses.find(conn_id);
                if (it != g_switch2_cmd_responses.end()) {
                    expected_cmd = it->second.command;
                    expected_sub = it->second.subcommand;
                    result = it->second.Accept(data, size);
                } else if (!sw2::IsCommandResponse(data, size)) {
                    result = sw2::AcceptResult::Malformed;
                }
            }

            if (result == sw2::AcceptResult::Accepted) {
                SW2_LOG_DATA_INFO(data, size, "[S4][CMD-RX] conn_id=%u cmd=0x%02X sub=0x%02X ack=0x%02X result=%s",
                    conn_id, data[0], data[3], data[5], sw2::AcceptResultName(result));
                return;
            }

            bool should_log = false;
            {
                std::scoped_lock lk(g_conn_map_lock);
                const auto it = g_connection_stats.find(conn_id);
                if (it != g_connection_stats.end()) {
                    should_log = ++it->second.responses_rejected <= MaxRejectedResponseLogs;
                }
            }
            if (should_log) {
                SW2_LOG_DATA_WARN(data, size, "[S4][DROP][CMD-RX] conn_id=%u result=%s awaiting_cmd=0x%02X awaiting_sub=0x%02X",
                    conn_id, sw2::AcceptResultName(result), expected_cmd, expected_sub);
            }
        }

        Result WaitForSwitch2CommandResponse(u32 conn_id, u32 timeout_ms, u8 *out_ack) {
            const u64 timeout_ns = static_cast<u64>(timeout_ms) * 1'000'000ull;
            const u64 end_ns = GetCurrentTimeNs() + timeout_ns;

            while (GetCurrentTimeNs() < end_ns) {
                {
                    std::scoped_lock lk(g_switch2_cmd_response_lock);
                    auto it = g_switch2_cmd_responses.find(conn_id);
                    if (it == g_switch2_cmd_responses.end() || !it->second.awaiting) {
                        // Disconnect erased or reconnect reset the transaction.
                        return Sw2Error(sw2::Error::StaleConnection);
                    }

                    if (it->second.completed) {
                        *out_ack = it->second.ack;
                        if (!sw2::IsSuccessfulAck(it->second.ack)) return Sw2Error(sw2::Error::AckRejected);
                        R_SUCCEED();
                    }
                }

                os::SleepThread(ams::TimeSpan::FromMilliSeconds(5));
            }

            return Sw2Error(sw2::Error::AckTimeout);
        }

        bool IsSwitch2CommandRequest(const bluetooth::HidReport *report) {
            return report->size >= 8 && report->data[1] == 0x91 && report->data[2] == 0x01;
        }

        bool ShouldWaitForSwitch2CommandResponse(const bluetooth::HidReport *report) {
            if (!IsSwitch2CommandRequest(report)) {
                return false;
            }

            switch (report->data[0]) {
                case 0x02:
                case 0x03:
                case 0x07:
                case 0x09:
                case 0x0A:
                case 0x0C:
                case 0x10:
                case 0x11:
                case 0x15:
                case 0x16:
                    return true;
                default:
                    return false;
            }
        }

        constexpr u8 GattPropertyWriteNoResponse = BtdrvGattCharacteristicProperty_WriteNoResponse;
        constexpr u8 GattPropertyWrite           = BtdrvGattCharacteristicProperty_Write;
        constexpr u8 GattPropertyNotify          = BtdrvGattCharacteristicProperty_Notify;
        constexpr u8 GattPropertyIndicate        = BtdrvGattCharacteristicProperty_Indicate;

        Result RegisterSwitch2GattNotification(u32 conn_id, const Switch2GattPath &path, const BtdrvGattAttributeUuid &char_uuid, const char *target) {
            BtdrvGattId char_id = {};
            BtdrvGattId descriptor_id = {};
            u8 properties = 0;
            R_TRY(LogGattStep(conn_id, target, "CHAR", btdrvGetGattFirstCharacteristic(conn_id, &path.service, path.primary, &char_uuid, &properties, &char_id)));
            if ((properties & (GattPropertyNotify | GattPropertyIndicate)) == 0) {
                SW2_LOG_WARN("[S3][WARN][GATT-%s-CHAR] conn_id=%u properties=0x%02X lack notify/indicate", target, conn_id, properties);
            } else {
                SW2_LOG_VERBOSE("[S3][GATT-%s-CHAR] conn_id=%u properties=0x%02X instance=%u", target, conn_id, properties, char_id.instance_id);
            }
            constexpr BtdrvGattAttributeUuid ccc_uuid = {2, {0x29, 0x02}};
            R_TRY(LogGattStep(conn_id, target, "CCC-DESC", btdrvGetGattFirstDescriptor(conn_id, &path.service, path.primary, &char_id, &ccc_uuid, &descriptor_id)));
            R_TRY(LogGattStep(conn_id, target, "DATA-PATH", btdrvRegisterGattManagedDataPath(&char_uuid)));
            R_TRY(LogGattStep(conn_id, target, "NOTIFY-REG", btdrvRegisterGattNotification(conn_id, path.primary, &path.service, &char_id)));
            constexpr u8 enable_notifications[] = {0x01, 0x00};
            // IPC success only means the request was queued; the first notification proves delivery.
            R_RETURN(LogGattStep(conn_id, target, "CCC-WRITE", btdrvWriteGattDescriptor(conn_id, path.primary, &path.service, &char_id, &descriptor_id, enable_notifications, sizeof(enable_notifications), 0)));
        }

        Result InitializeSwitch2Gatt(const PendingSwitch2InitRequest &request) {
            const auto input_uuid = GetSwitch2InputCharUuid(request.type);
            if (!input_uuid) {
                SW2_LOG_WARN("[S3][FAIL][GATT-TYPE] conn_id=%u unsupported type=%u", request.conn_id, static_cast<u32>(request.type));
                return Sw2Error(sw2::Error::UnsupportedType);
            }

            SW2_LOG_INFO("[S3][GATT-BEGIN] conn_id=%u type=%s generation=%llu",
                request.conn_id, GetSwitch2ControllerTypeName(request.type), static_cast<unsigned long long>(request.generation));

            R_TRY(LogGattStep(request.conn_id, "BTM", "INIT", btmInitialize()));
            ON_SCOPE_EXIT { btmExit(); };
            BtmGattService service = {};
            bool found = false;
            // btm owns service discovery. Wait for its cache, without taking over
            // the shared driver's discovery or scan lifecycle.
            constexpr unsigned int ServiceLookupAttempts = 100;
            constexpr unsigned int ServiceLookupIntervalMs = 20;
            unsigned int attempt = 0;
            for (; attempt < ServiceLookupAttempts; ++attempt) {
                if (!IsCurrentConnection(request)) return LogStaleConnection(request, "SERVICE-LOOKUP");
                const Result rc = btmGetGattService(request.conn_id, &Switch2InputServiceUuid, &service, &found);
                if (R_FAILED(rc)) return LogGattStep(request.conn_id, "SERVICE", "LOOKUP", rc);
                if (found) break;
                os::SleepThread(ams::TimeSpan::FromMilliSeconds(ServiceLookupIntervalMs));
            }
            if (!found) {
                SW2_LOG_WARN("[S3][FAIL][GATT-SERVICE-LOOKUP] conn_id=%u service ab7de9be... absent from btm cache after %u attempts (%u ms)",
                    request.conn_id, ServiceLookupAttempts, ServiceLookupAttempts * ServiceLookupIntervalMs);
                return Sw2Error(sw2::Error::ServiceNotFound);
            }
            SW2_LOG_INFO("[S3][OK][GATT-SERVICE-LOOKUP] conn_id=%u attempts=%u handle=0x%04X end=0x%04X instance=%u primary=%u",
                request.conn_id, attempt + 1, service.handle, service.end_group_handle, service.instance_id, service.primary_service);
            if (service.instance_id > 0xFF || !UuidEquals(service.uuid, Switch2InputServiceUuid)) {
                SW2_LOG_WARN("[S3][FAIL][GATT-SERVICE-VALIDATE] conn_id=%u instance=%u uuid_size=%u (btdrv needs u8 instance and matching UUID)",
                    request.conn_id, service.instance_id, service.uuid.size);
                return Sw2Error(sw2::Error::ServiceNotFound);
            }
            Switch2GattPath path = {};
            path.service = MakeGattId(Switch2InputServiceUuid);
            path.service.instance_id = static_cast<u8>(service.instance_id);
            path.primary = service.primary_service != 0;
            path.type = request.type;
            u8 properties = 0;
            R_TRY(LogGattStep(request.conn_id, "CMD", "CHAR", btdrvGetGattFirstCharacteristic(request.conn_id, &path.service, path.primary, &Switch2CommandCharUuid, &properties, &path.command)));
            if ((properties & (GattPropertyWrite | GattPropertyWriteNoResponse)) == 0) {
                SW2_LOG_WARN("[S3][WARN][GATT-CMD-CHAR] conn_id=%u properties=0x%02X lack write", request.conn_id, properties);
            } else {
                SW2_LOG_VERBOSE("[S3][GATT-CMD-CHAR] conn_id=%u properties=0x%02X instance=%u", request.conn_id, properties, path.command.instance_id);
            }
            R_TRY(RegisterSwitch2GattNotification(request.conn_id, path, Switch2CommandResponse1CharUuid, "RESP"));
            // Use the model's default format only. Do not subscribe to competing
            // streams and infer format from whichever packet arrived first.
            R_TRY(RegisterSwitch2GattNotification(request.conn_id, path, *input_uuid, "INPUT"));
            if (!IsCurrentConnection(request)) return LogStaleConnection(request, "PATH-PUBLISH");
            {
                std::scoped_lock lk(g_conn_map_lock);
                const auto it = g_ble_conn_ids.find(request.address);
                if (it == g_ble_conn_ids.end() || it->second != request.conn_id) return LogStaleConnection(request, "PATH-PUBLISH");
                g_switch2_gatt_paths[request.conn_id] = path;
            }
            // The handler's Initialize() runs the [S4] command bootstrap synchronously.
            R_TRY(LogGattStep(request.conn_id, "HANDLER", "ATTACH", controller::AttachHandler(request.address)));
            if (!IsCurrentConnection(request)) {
                controller::RemoveHandler(request.address);
                return LogStaleConnection(request, "HANDLER-ATTACH");
            }
            {
                std::scoped_lock lk(g_conn_map_lock);
                const auto it = g_switch2_gatt_paths.find(request.conn_id);
                if (it == g_switch2_gatt_paths.end()) return LogStaleConnection(request, "READY");
                it->second.ready = true;
            }

            R_SUCCEED();
        }

        Result WriteSwitch2GattDataReportImpl(u32 conn_id, const bluetooth::HidReport *report) {
            Switch2GattPath path = {};
            {
                std::scoped_lock lk(g_conn_map_lock);
                const auto it = g_switch2_gatt_paths.find(conn_id);
                if (it == g_switch2_gatt_paths.end()) return Sw2Error(sw2::Error::GattPathMissing);
                path = it->second;
            }
            return btdrvWriteGattCharacteristic(conn_id, path.primary, &path.service, &path.command, report->data, report->size, 0, false);
        }

        // Switch 2 advertisement matching follows documentation/bluetooth_interface.md:
        // Manufacturer data starts with company id 0x0553, marker bytes 01 00 03,
        // vendor id 0x057E, then a product id identifying controller type.
        constexpr u16 Switch2ManufacturerCompanyId = 0x0553;
        constexpr u16 Switch2ManufacturerVendorId = 0x057E;
        constexpr u8 Switch2ManufacturerPrefix[] = { 0x01, 0x00, 0x03 };

        constexpr size_t Switch2ManufacturerMinSize = 9;
        constexpr u16 Switch2PidJoyConL = 0x2060;
        constexpr u16 Switch2PidJoyConR = 0x2061;
        constexpr u16 Switch2PidPro = 0x2062;
        constexpr u16 Switch2PidNsoGc = 0x2064;

        bool MatchSwitch2ManufacturerData(const auto &ad) {
            if (ad.type != 0xFF || ad.size < Switch2ManufacturerMinSize) {
                return false;
            }

            const u16 company_id = static_cast<u16>(ad.data[0] | (static_cast<u16>(ad.data[1]) << 8));
            if (company_id != Switch2ManufacturerCompanyId) {
                return false;
            }

            if (std::memcmp(ad.data + 2, Switch2ManufacturerPrefix, sizeof(Switch2ManufacturerPrefix)) != 0) {
                return false;
            }

            const u16 vendor_id = static_cast<u16>(ad.data[5] | (static_cast<u16>(ad.data[6]) << 8));
            return vendor_id == Switch2ManufacturerVendorId;
        }

        controller::ControllerType DecodeSwitch2TypeFromManufacturerData(const u8 *data, size_t size) {
            if (size < Switch2ManufacturerMinSize) {
                return controller::ControllerType_Unknown;
            }

            const u16 company_id = static_cast<u16>(data[0] | (static_cast<u16>(data[1]) << 8));
            if (company_id != Switch2ManufacturerCompanyId) {
                return controller::ControllerType_Unknown;
            }

            if (std::memcmp(data + 2, Switch2ManufacturerPrefix, sizeof(Switch2ManufacturerPrefix)) != 0) {
                return controller::ControllerType_Unknown;
            }

            const u16 vendor_id = static_cast<u16>(data[5] | (static_cast<u16>(data[6]) << 8));
            if (vendor_id != Switch2ManufacturerVendorId) {
                return controller::ControllerType_Unknown;
            }

            const u16 product_id = static_cast<u16>(data[7] | (static_cast<u16>(data[8]) << 8));
            switch (product_id) {
                case Switch2PidJoyConL: return controller::ControllerType_Switch2JoyConL;
                case Switch2PidJoyConR: return controller::ControllerType_Switch2JoyConR;
                case Switch2PidPro:     return controller::ControllerType_Switch2ProController;
                case Switch2PidNsoGc:   return controller::ControllerType_Switch2NSOGCController;
                default:                return controller::ControllerType_Unknown;
            }
        }

        // IMPORTANT: We deliberately do NOT call btdrvStartBleScan/btdrvStopBleScan,
        // btdrvEnableBleScanFilter or btmBleConnect from this managed-event callback.
        // That callback runs in the shared btdrv event context and re-entering BLE
        // control APIs from it can race btm-sysmodule. We only observe results here
        // and queue connect requests to a dedicated worker thread.

        void HandleScanFilterEvent(const bluetooth::BleEventInfo &info) {
            // Other components may modify scan filters. We only observe the event
            // here to avoid re-entering the BLE control path from a managed callback.
            (void)info;
        }

        void HandleScanResultEvent(const bluetooth::BleEventInfo &info) {
            const auto &sr = info.scan_result;

            // Passive observation only. btm-sysmodule owns the scan lifecycle; we never
            // restart it or initiate connections from this callback (that destabilises
            // the system). We log every result so we can confirm whether the console's
            // own scan ever surfaces a Switch 2 advertisement to this MITM.
            //
            // status: 0xFF=scan started, 2=new device found, 1=scan complete.
            if (sr.result != 0 || sr.status != 2) {
                SW2_LOG_VERBOSE("[S1][SCAN-STATUS] status=%s(0x%02X) result=0x%08X", GetScanStatusName(sr.status), sr.status, sr.result);
                return;
            }

            // Rate-limited: the first results prove the MITM sees the console's scan,
            // then periodic summaries show it keeps running without flooding the SD log.
            ++g_scan_results_seen;
            if (g_scan_results_seen <= MaxScanResultLogs) {
                SW2_LOG_VERBOSE(
                    "[S1][SCAN-MAC] mac=%02X:%02X:%02X:%02X:%02X:%02X rssi=%d ads=%u addr_type=%u",
                    sr.address.address[0], sr.address.address[1], sr.address.address[2],
                    sr.address.address[3], sr.address.address[4], sr.address.address[5],
                    sr.rssi, static_cast<u32>(sr.count), sr.ble_addr_type
                );
            } else if ((g_scan_results_seen % ScanSummaryInterval) == 0) {
                SW2_LOG_INFO("[S1][SCAN-SUMMARY] results_seen=%u", g_scan_results_seen);
            }

            controller::ControllerType detected = controller::ControllerType_Unknown;

            for (size_t i = 0; i < sr.count && i < std::size(sr.ad_list); ++i) {
                const auto &ad = sr.ad_list[i];
                if (ad.size == 0 || ad.size > sizeof(ad.data)) {
                    continue;
                }

                if (MatchSwitch2ManufacturerData(ad)) {
                    controller::ControllerType t = DecodeSwitch2TypeFromManufacturerData(ad.data, ad.size);
                    if (t != controller::ControllerType_Unknown) {
                        detected = t;
                        break;
                    }
                }

                // Nintendo company id but unexpected layout/PID: log so a format change is visible.
                if (ad.type == 0xFF && ad.size >= 2 &&
                    static_cast<u16>(ad.data[0] | (static_cast<u16>(ad.data[1]) << 8)) == Switch2ManufacturerCompanyId &&
                    g_nonmatching_0553_logs < MaxNonmatching0553Logs) {
                    ++g_nonmatching_0553_logs;
                    SW2_LOG_DATA_WARN(ad.data, ad.size,
                        "[S2][SKIP][ADV-0553] mac=%02X:%02X:%02X:%02X:%02X:%02X manufacturer data not a known Switch 2 layout/PID",
                        sr.address.address[0], sr.address.address[1], sr.address.address[2],
                        sr.address.address[3], sr.address.address[4], sr.address.address[5]);
                }
            }

            if (detected != controller::ControllerType_Unknown) {
                u32 list_index = 0;
                bool is_new = false;
                RegisterSwitch2DiscoveryListEntry(sr.address, detected, &list_index, &is_new);
                if (list_index == 0) {
                    static bool s_logged_list_full = false; // event thread only
                    if (!s_logged_list_full) {
                        s_logged_list_full = true;
                        SW2_LOG_WARN("[S2][FAIL][FOUND] discovery list full (32); ignoring new controllers");
                    }
                    return;
                }
                controller::RegisterDiscoveredSwitch2Controller(sr.address, detected);

                if (is_new) {
                    SW2_LOG_INFO(
                        "[S2][OK][FOUND-%02u] type=%s mac=%02X:%02X:%02X:%02X:%02X:%02X rssi=%d addr_type=%u",
                        list_index,
                        GetSwitch2ControllerTypeName(detected),
                        sr.address.address[0], sr.address.address[1], sr.address.address[2],
                        sr.address.address[3], sr.address.address[4], sr.address.address[5],
                        sr.rssi, sr.ble_addr_type
                    );
                }

                QueueSwitch2ConnectRequest(sr.address, detected, "ScanResult");
            }
        }

        void HandleClientConnectionEvent(const bluetooth::BleEventInfo &info) {
            const auto &cc = info.client_connection;

            if (cc.result != 0 && cc.status != 2) {
                SW2_LOG_WARN("[S3][FAIL][CONNECTION-EVENT] result=0x%08X status=%u conn_id=%u mac=%02X:%02X:%02X:%02X:%02X:%02X",
                    cc.result, cc.status, cc.conn_id,
                    cc.address.address[0], cc.address.address[1], cc.address.address[2],
                    cc.address.address[3], cc.address.address[4], cc.address.address[5]);
                return;
            }
            if (cc.status == 0) {
                controller::ControllerType sw2_type = controller::GetDiscoveredSwitch2ControllerType(cc.address);
                if (sw2_type == controller::ControllerType_Unknown) sw2_type = ResolveSwitch2TypeFromPairedInfo(cc.address);
                // Do not route or remove handlers for unrelated BLE accessories.
                if (!IsSwitch2ControllerType(sw2_type)) {
                    SW2_LOG_INFO("[S3][SKIP][CONNECTED] conn_id=%u mac=%02X:%02X:%02X:%02X:%02X:%02X not identified as Switch 2 (no discovery entry or paired info)",
                        cc.conn_id,
                        cc.address.address[0], cc.address.address[1], cc.address.address[2],
                        cc.address.address[3], cc.address.address[4], cc.address.address[5]);
                    return;
                }
                controller::RegisterDiscoveredSwitch2Controller(cc.address, sw2_type);
                u64 generation = 0;
                bool rejected = false;
                {
                    std::scoped_lock lk(g_conn_map_lock);
                    if (g_conn_map.find(cc.conn_id) != g_conn_map.end() || g_conn_map.size() >= 4) {
                        rejected = true;
                    } else {
                        g_conn_map[cc.conn_id] = cc.address;
                        g_ble_conn_ids[cc.address] = cc.conn_id;
                        generation = ++g_next_connection_generation;
                        g_connection_generations[cc.conn_id] = generation;
                        g_connection_stats[cc.conn_id] = {};
                    }
                }
                if (rejected) {
                    SW2_LOG_WARN("[S3][FAIL][CONNECTED] conn_id=%u duplicate conn_id or 4 connections already tracked", cc.conn_id);
                    return;
                }
                ResetSwitch2CommandState(cc.conn_id);

                SW2_LOG_INFO(
                    "[S3][OK][CONNECTED] conn_id=%u type=%s generation=%llu mac=%02X:%02X:%02X:%02X:%02X:%02X",
                    cc.conn_id, GetSwitch2ControllerTypeName(sw2_type), static_cast<unsigned long long>(generation),
                    cc.address.address[0], cc.address.address[1], cc.address.address[2],
                    cc.address.address[3], cc.address.address[4], cc.address.address[5]
                );

                QueueSwitch2Initialize({cc.address, cc.conn_id, sw2_type, generation});
            } else if (cc.status == 2) {
                Switch2ConnectionStats stats = {};
                {
                    std::scoped_lock lk(g_conn_map_lock);
                    const auto it = g_ble_conn_ids.find(cc.address);
                    if (it == g_ble_conn_ids.end() || it->second != cc.conn_id) return;
                    g_conn_map.erase(cc.conn_id);
                    g_ble_conn_ids.erase(it);
                    g_switch2_gatt_paths.erase(cc.conn_id);
                    g_connection_generations.erase(cc.conn_id);
                    const auto stats_it = g_connection_stats.find(cc.conn_id);
                    if (stats_it != g_connection_stats.end()) {
                        stats = stats_it->second;
                        g_connection_stats.erase(stats_it);
                    }
                }
                ClearSwitch2CommandState(cc.conn_id);

                SW2_LOG_INFO(
                    "[S3][DISCONNECTED] conn_id=%u mac=%02X:%02X:%02X:%02X:%02X:%02X reason=0x%04X result=0x%08X",
                    cc.conn_id,
                    cc.address.address[0], cc.address.address[1], cc.address.address[2],
                    cc.address.address[3], cc.address.address[4], cc.address.address[5],
                    cc.reason, cc.result
                );
                LogConnectionStats(cc.conn_id, stats);

                controller::RemoveHandler(cc.address);
            } else {
                SW2_LOG_VERBOSE("[S3][CONNECTION-EVENT] conn_id=%u unhandled status=%u", cc.conn_id, cc.status);
            }
        }

        // Event thread only: scratch for UUID text in rate-limited drop logs.
        char g_uuid_text[40];

        void HandleClientNotifyEvent(const bluetooth::BleEventInfo &info) {
            const auto &cn = info.client_notify;
            bluetooth::Address addr = {};
            bool have_addr = false;

            {
                std::scoped_lock lk(g_conn_map_lock);
                auto it = g_conn_map.find(cn.conn_id);
                if (it != g_conn_map.end()) {
                    addr = it->second;
                    have_addr = true;
                    const auto stats = g_connection_stats.find(cn.conn_id);
                    if (stats != g_connection_stats.end()) ++stats->second.notifications;
                }
            }

            if (!have_addr) {
                // Notifications of other BLE peripherals are expected; log only a few.
                if (g_untracked_notify_logs < MaxUntrackedNotifyLogs) {
                    ++g_untracked_notify_logs;
                    SW2_LOG_VERBOSE("[S5][SKIP][NOTIFY-UNTRACKED] conn_id=%u size=%u (not a tracked Switch 2 connection)", cn.conn_id, cn.size);
                }
                return;
            }

            if (cn.result != 0 || cn.size == 0 || cn.size > sizeof(cn.data) || (cn.type != 4 && cn.type != 5)) {
                SW2_LOG_WARN("[S5][DROP][NOTIFY-EVENT] conn_id=%u result=0x%08X type=%u size=%u", cn.conn_id, cn.result, cn.type, cn.size);
                return;
            }

            if (!UuidEquals(cn.serv_uuid, Switch2InputServiceUuid)) {
                if (CountNotifyDrop(cn.conn_id, NotifyDrop_ForeignService)) {
                    FormatGattUuid(g_uuid_text, sizeof(g_uuid_text), cn.serv_uuid);
                    SW2_LOG_DATA_INFO(cn.data, cn.size, "[S5][DROP][%s] conn_id=%u service=%s (first only)",
                        NotifyDropNames[NotifyDrop_ForeignService], cn.conn_id, g_uuid_text);
                }
                return;
            }

            if (UuidEquals(cn.char_uuid, Switch2CommandResponse1CharUuid)) {
                {
                    std::scoped_lock lk(g_conn_map_lock);
                    const auto stats = g_connection_stats.find(cn.conn_id);
                    if (stats != g_connection_stats.end()) ++stats->second.responses;
                }
                RecordSwitch2CommandResponse(cn.conn_id, cn.data, cn.size);
                return;
            }

            controller::ControllerType notify_type = controller::ControllerType_Unknown;
            NotifyDrop drop = NotifyDrop_Count;
            {
                std::scoped_lock lk(g_conn_map_lock);
                const auto it = g_switch2_gatt_paths.find(cn.conn_id);
                if (it == g_switch2_gatt_paths.end() || !it->second.ready) {
                    drop = NotifyDrop_NotReady;
                } else {
                    notify_type = it->second.type;
                    const auto expected = GetSwitch2InputCharUuid(notify_type);
                    if (expected == nullptr || !UuidEquals(cn.char_uuid, *expected)) drop = NotifyDrop_UnexpectedChar;
                }
            }
            if (drop == NotifyDrop_NotReady) {
                // Expected while the command bootstrap is still running.
                if (CountNotifyDrop(cn.conn_id, drop)) {
                    SW2_LOG_DATA_VERBOSE(cn.data, cn.size, "[S5][SKIP][%s] conn_id=%u input before GATT setup finished (first only)",
                        NotifyDropNames[drop], cn.conn_id);
                }
                return;
            }
            if (drop == NotifyDrop_UnexpectedChar) {
                if (CountNotifyDrop(cn.conn_id, drop)) {
                    FormatGattUuid(g_uuid_text, sizeof(g_uuid_text), cn.char_uuid);
                    SW2_LOG_DATA_WARN(cn.data, cn.size, "[S5][DROP][%s] conn_id=%u char=%s type=%s (first only)",
                        NotifyDropNames[drop], cn.conn_id, g_uuid_text, GetSwitch2ControllerTypeName(notify_type));
                }
                return;
            }

            auto device = controller::LocateHandler(addr);
            if (!device) {
                // Initialization belongs to the worker, never this event thread.
                if (CountNotifyDrop(cn.conn_id, NotifyDrop_NoHandler)) {
                    SW2_LOG_WARN("[S5][DROP][%s] conn_id=%u ready path without controller handler (first only)",
                        NotifyDropNames[NotifyDrop_NoHandler], cn.conn_id);
                }
                return;
            }

            u8 report_id = 0;
            switch (notify_type) {
                case controller::ControllerType_Switch2JoyConL: report_id = 0x07; break;
                case controller::ControllerType_Switch2JoyConR: report_id = 0x08; break;
                case controller::ControllerType_Switch2ProController: report_id = 0x09; break;
                case controller::ControllerType_Switch2NSOGCController: report_id = 0x0A; break;
                default: return;
            }
            if (cn.size < sw2::MinimumPayloadSize(report_id)) {
                if (CountNotifyDrop(cn.conn_id, NotifyDrop_TooShort)) {
                    SW2_LOG_DATA_WARN(cn.data, cn.size, "[S5][DROP][%s] conn_id=%u report=0x%02X minimum=%u (check MTU) (first only)",
                        NotifyDropNames[NotifyDrop_TooShort], cn.conn_id, report_id,
                        static_cast<u32>(sw2::MinimumPayloadSize(report_id)));
                }
                return;
            }

            // This callback has one consumer (mc::EventThread). Avoid two large
            // overlapping automatic report buffers on its small stack.
            static bluetooth::HidReportEventInfo event_info = {};
            event_info.data_report.v9.res = 0;
            event_info.data_report.v9.proto_mode = 0;
            event_info.data_report.v9.addr = addr;
            auto &report = event_info.data_report.v9.report;
            report.size = cn.size + 1;
            report.data[0] = report_id;
            std::memcpy(report.data + 1, cn.data, cn.size);

            const Result rc = device->HandleDataReportEvent(&event_info);
            if (R_FAILED(rc)) {
                if (CountNotifyDrop(cn.conn_id, NotifyDrop_HandlerError)) {
                    SW2_LOG_WARN("[S5][DROP][%s] conn_id=%u report=0x%02X forwarding to HID event buffer failed " SW2_RC_FMT " (first only)",
                        NotifyDropNames[NotifyDrop_HandlerError], cn.conn_id, report_id, SW2_RC_ARGS(rc));
                }
                return;
            }

            bool log_first = false;
            u32 delivered = 0;
            {
                std::scoped_lock lk(g_conn_map_lock);
                const auto stats = g_connection_stats.find(cn.conn_id);
                if (stats != g_connection_stats.end()) {
                    delivered = ++stats->second.delivered;
                    log_first = !stats->second.first_input_logged;
                    stats->second.first_input_logged = true;
                }
            }
            if (log_first) {
                SW2_LOG_DATA_INFO(cn.data, cn.size, "[S5][OK][INPUT-FIRST] conn_id=%u report=0x%02X forwarded to HID event buffer (Horizon registration not verified)",
                    cn.conn_id, report_id);
            } else if (delivered != 0 && (delivered % InputAliveLogInterval) == 0) {
                SW2_LOG_VERBOSE("[S5][INPUT-ALIVE] conn_id=%u delivered=%u", cn.conn_id, delivered);
            }
        }

        // Logs BLE events that the bridge does not act on, for tracked connections only.
        void LogOtherBleEvent(bluetooth::BleEventType type, const bluetooth::BleEventInfo &info) {
            auto tracked = [](u32 conn_id) {
                std::scoped_lock lk(g_conn_map_lock);
                return g_conn_map.find(conn_id) != g_conn_map.end();
            };

            switch (type) {
                case BtdrvBleEventType_ConnectionUpdate:
                    if (tracked(info.connection_update.conn_id)) {
                        SW2_LOG_INFO("[S1][BLE-EVENT] ConnectionUpdate conn_id=%u result=0x%08X interval=%u latency=%u timeout=%u",
                            info.connection_update.conn_id, info.connection_update.result, info.connection_update.conn_interval,
                            info.connection_update.conn_latency, info.connection_update.supervision_tout);
                    }
                    break;
                case BtdrvBleEventType_ClientConfigureMtu:
                    if (tracked(info.client_configure_mtu.conn_id)) {
                        SW2_LOG_INFO("[S1][BLE-EVENT] ClientConfigureMtu conn_id=%u result=0x%08X mtu=%u",
                            info.client_configure_mtu.conn_id, info.client_configure_mtu.result, info.client_configure_mtu.mtu);
                    }
                    break;
                case BtdrvBleEventType_ClientCacheSave:
                    if (tracked(info.client_cache_save.conn_id)) {
                        SW2_LOG_INFO("[S1][BLE-EVENT] ClientCacheSave conn_id=%u result=0x%08X attributes=%u",
                            info.client_cache_save.conn_id, info.client_cache_save.result, info.client_cache_save.count);
                    }
                    break;
                case BtdrvBleEventType_ClientCacheLoad:
                    if (tracked(info.client_cache_load.conn_id)) {
                        SW2_LOG_INFO("[S1][BLE-EVENT] ClientCacheLoad conn_id=%u result=0x%08X",
                            info.client_cache_load.conn_id, info.client_cache_load.result);
                    }
                    break;
                default:
                    SW2_LOG_VERBOSE("[S1][BLE-EVENT] %s(%u)", GetBleEventTypeName(type), static_cast<u32>(type));
                    break;
            }
        }

    }

    bool IsInitialized() {
        return g_init_event.TryWait();
    }

    void SignalInitialized() {
        g_init_event.Signal();
        if (ams::mitm::GetGlobalConfig()->bluetooth.enable_switch2_experimental) StartSwitch2ConnectWorkerIfNeeded();
    }

    void WaitInitialized() {
        g_init_event.Wait();
    }

    os::SystemEvent *GetSystemEvent() {
        return &g_system_event;
    }

    os::SystemEvent *GetForwardEvent() {
        return &g_system_event_fwd;
    }

    os::SystemEvent *GetUserForwardEvent() {
        return &g_system_event_user_fwd;
    }

    Result GetEventInfo(bluetooth::BleEventType *type, void *buffer, size_t size) {
        std::scoped_lock lk(g_event_data_lock);

        *type = g_current_event_type;
        std::memcpy(buffer, &g_event_info, std::min(size, sizeof(g_event_info)));

        g_data_read_event.Signal();
        
        R_SUCCEED();
    }

    bool HasSwitch2GattConnection(const bluetooth::Address &address, u32 *out_conn_id) {
        if (controller::GetDiscoveredSwitch2ControllerType(address) == controller::ControllerType_Unknown) {
            return false;
        }

        return HasBleConnection(address, out_conn_id);
    }

    Result WriteSwitch2GattDataReport(const bluetooth::Address &address, const bluetooth::HidReport *report) {
        if (report == nullptr) {
            SW2_LOG_WARN("[S4][FAIL][CMD-INVALID] null report");
            return Sw2Error(sw2::Error::InvalidCommand);
        }
        if (report->size > sizeof(report->data) || !IsSwitch2CommandRequest(report) || report->size != 8u + report->data[5]) {
            SW2_LOG_DATA_WARN(report->data, std::min<size_t>(report->size, sizeof(report->data)),
                "[S4][FAIL][CMD-INVALID] frame is not a well-formed 91 01 command (expected size 8 + data[5])");
            return Sw2Error(sw2::Error::InvalidCommand);
        }
        const u8 cmd = report->data[0];
        const u8 sub = report->data[3];
        // Arm BEFORE the IPC write: a fast response may arrive before it returns.
        // Serialize commands because this protocol has no transaction identifier.
        std::scoped_lock write_lock(g_switch2_write_lock);
        u32 conn_id = 0;
        if (!HasBleConnection(address, &conn_id)) {
            SW2_LOG_WARN("[S4][FAIL][CMD-WRITE] cmd=0x%02X sub=0x%02X no BLE connection for mac=%02X:%02X:%02X:%02X:%02X:%02X",
                cmd, sub,
                address.address[0], address.address[1], address.address[2],
                address.address[3], address.address[4], address.address[5]);
            return Sw2Error(sw2::Error::StaleConnection);
        }

        const bool wait_response = ShouldWaitForSwitch2CommandResponse(report);
        bool have_state = false;
        {
            std::scoped_lock lk(g_switch2_cmd_response_lock);
            const auto it = g_switch2_cmd_responses.find(conn_id);
            if (it != g_switch2_cmd_responses.end()) {
                it->second = {wait_response, false, cmd, sub, 0};
                have_state = true;
            }
        }
        if (!have_state) {
            SW2_LOG_WARN("[S4][FAIL][CMD-WRITE] conn_id=%u cmd=0x%02X sub=0x%02X no command state (connection being torn down?)", conn_id, cmd, sub);
            CountCommand(conn_id, false);
            return Sw2Error(sw2::Error::CommandStateMissing);
        }
        ON_SCOPE_EXIT {
            std::scoped_lock lk(g_switch2_cmd_response_lock);
            const auto it = g_switch2_cmd_responses.find(conn_id);
            if (it != g_switch2_cmd_responses.end()) it->second.awaiting = false;
        };

        SW2_LOG_DATA_INFO(report->data, report->size, "[S4][CMD-TX] conn_id=%u cmd=0x%02X sub=0x%02X wait_ack=%u",
            conn_id, cmd, sub, wait_response ? 1u : 0u);

        const u64 start_ns = GetCurrentTimeNs();
        const Result rc = WriteSwitch2GattDataReportImpl(conn_id, report);
        if (R_FAILED(rc)) {
            SW2_LOG_WARN("[S4][FAIL][CMD-WRITE] conn_id=%u cmd=0x%02X sub=0x%02X " SW2_RC_FMT, conn_id, cmd, sub, SW2_RC_ARGS(rc));
            CountCommand(conn_id, false);
            return rc;
        }

        if (!wait_response) {
            SW2_LOG_VERBOSE("[S4][OK][CMD-WRITE] conn_id=%u cmd=0x%02X sub=0x%02X no ack expected", conn_id, cmd, sub);
            CountCommand(conn_id, true);
            R_SUCCEED();
        }

        u8 ack = 0;
        const Result response_rc = WaitForSwitch2CommandResponse(conn_id, 1000, &ack);
        const u32 elapsed_ms = static_cast<u32>((GetCurrentTimeNs() - start_ns) / 1'000'000ull);
        if (R_FAILED(response_rc)) {
            SW2_LOG_WARN("[S4][FAIL][CMD-ACK] conn_id=%u cmd=0x%02X sub=0x%02X ack=0x%02X elapsed_ms=%u " SW2_RC_FMT,
                conn_id, cmd, sub, ack, elapsed_ms, SW2_RC_ARGS(response_rc));
            CountCommand(conn_id, false);
            return response_rc;
        }

        SW2_LOG_INFO("[S4][OK][CMD-ACK] conn_id=%u cmd=0x%02X sub=0x%02X ack=0x%02X elapsed_ms=%u", conn_id, cmd, sub, ack, elapsed_ms);
        CountCommand(conn_id, true);
        R_SUCCEED();
    }

    void HandleEvent() {
        {
            std::scoped_lock lk(g_event_data_lock);
            R_ABORT_UNLESS(btdrvGetBleManagedEventInfo(&g_event_info, sizeof(bluetooth::BleEventInfo), &g_current_event_type));
        }

        if (ams::mitm::GetGlobalConfig()->bluetooth.enable_switch2_experimental) switch (g_current_event_type) {
            case BtdrvBleEventType_ScanResult:
                HandleScanResultEvent(g_event_info);
                break;
            case BtdrvBleEventType_ScanFilter:
                HandleScanFilterEvent(g_event_info);
                break;
            case BtdrvBleEventType_ClientConnection:
                HandleClientConnectionEvent(g_event_info);
                break;
            case BtdrvBleEventType_ClientNotify:
                HandleClientNotifyEvent(g_event_info);
                break;
            default:
                LogOtherBleEvent(g_current_event_type, g_event_info);
                break;
        }

        if (!g_redirect_ble_events) {
            g_system_event_fwd.Signal();
            g_data_read_event.Wait();
        }

        if (g_system_event_user_fwd.GetBase()->state) {
            g_system_event_user_fwd.Signal();
        }
    }

}

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

        struct PendingSwitch2ConnectRequest {
            bluetooth::Address address;
            controller::ControllerType type;
        };

        struct Switch2CommandResponseState {
            u64 sequence;
            u8 last_cmd;
            u8 last_sub;
            u8 last_status;
            u8 last_ack;
        };

        constinit os::SdkMutex g_switch2_connect_queue_lock;
        os::Event g_switch2_connect_queue_event(os::EventClearMode_AutoClear);
        std::map<bluetooth::Address, PendingSwitch2ConnectRequest, AddressCompare> g_pending_switch2_connect_requests;
        std::map<bluetooth::Address, u64, AddressCompare> g_last_switch2_connect_attempt_ns;

        constinit os::SdkMutex g_switch2_cmd_response_lock;
        std::map<u32, Switch2CommandResponseState> g_switch2_cmd_responses;
        std::map<u32, bool> g_switch2_prefer_universal_input;

        constexpr s32 Switch2ConnectWorkerThreadPriority = 18;
        constexpr size_t Switch2ConnectWorkerThreadStackSize = 0x3000;
        alignas(os::ThreadStackAlignment) constinit u8 g_switch2_connect_worker_thread_stack[Switch2ConnectWorkerThreadStackSize];
        constinit os::ThreadType g_switch2_connect_worker_thread;
        constinit bool g_switch2_connect_worker_started = false;

        constexpr u64 Switch2ConnectRetryIntervalNs = 2'000'000'000ull;

        constexpr BtdrvGattAttributeUuid Switch2InputServiceUuid = { 16, { 0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad, 0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd0 } };
        constexpr BtdrvGattAttributeUuid Switch2InputReport0x05CharUuid = { 16, { 0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad, 0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd2 } };
        constexpr BtdrvGattAttributeUuid Switch2JoyConLInputCharUuid = { 16, { 0xcc, 0x1b, 0xbb, 0xb5, 0x73, 0x54, 0x4d, 0x32, 0xa7, 0x16, 0xa8, 0x1c, 0xb2, 0x41, 0xa3, 0x2a } };
        constexpr BtdrvGattAttributeUuid Switch2JoyConRInputCharUuid = { 16, { 0xd5, 0xa9, 0xe0, 0x1e, 0x2f, 0xfc, 0x4c, 0xca, 0xb2, 0x0c, 0x8b, 0x67, 0x14, 0x2b, 0xf4, 0x42 } };
        constexpr BtdrvGattAttributeUuid Switch2ProInputCharUuid = { 16, { 0x74, 0x92, 0x86, 0x6c, 0xec, 0x3e, 0x46, 0x19, 0x82, 0x58, 0x32, 0x75, 0x5f, 0xfc, 0xc0, 0xf8 } };
        constexpr BtdrvGattAttributeUuid Switch2NsoGcInputCharUuid = { 16, { 0x82, 0x61, 0xcb, 0xa1, 0x94, 0x35, 0x42, 0x0c, 0x84, 0xd6, 0xf0, 0xc7, 0x5a, 0x2c, 0x8e, 0x4d } };
        constexpr BtdrvGattAttributeUuid Switch2CommandCharUuid = { 16, { 0x64, 0x9d, 0x4a, 0xc9, 0x8e, 0xb7, 0x4e, 0x6c, 0xaf, 0x44, 0x1e, 0xa5, 0x4f, 0xe5, 0xf0, 0x05 } };
        constexpr BtdrvGattAttributeUuid Switch2CommandResponse1CharUuid = { 16, { 0xc7, 0x65, 0xa9, 0x61, 0xd9, 0xd8, 0x4d, 0x36, 0xa2, 0x0a, 0x53, 0x15, 0xb1, 0x11, 0x83, 0x6a } };

        const char *GetBleEventTypeName(const bluetooth::BleEventType type) {
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

        void FormatGattUuid(char *buf, size_t bufsz, const BtdrvGattAttributeUuid &uuid) {
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
            return lhs.size == rhs.size && std::memcmp(lhs.uuid, rhs.uuid, lhs.size) == 0;
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

        controller::ControllerType DecodeSwitch2TypeFromInputUuid(const BtdrvGattAttributeUuid &uuid) {
            if (UuidEquals(uuid, Switch2JoyConLInputCharUuid)) return controller::ControllerType_Switch2JoyConL;
            if (UuidEquals(uuid, Switch2JoyConRInputCharUuid)) return controller::ControllerType_Switch2JoyConR;
            if (UuidEquals(uuid, Switch2ProInputCharUuid)) return controller::ControllerType_Switch2ProController;
            if (UuidEquals(uuid, Switch2NsoGcInputCharUuid)) return controller::ControllerType_Switch2NSOGCController;
            return controller::ControllerType_Unknown;
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

        controller::ControllerType ResolveSwitch2TypeFromPairedInfo(const bluetooth::Address &address) {
            bluetooth::DevicesSettings device_settings = {};
            if (R_FAILED(btdrvGetPairedDeviceInfo(address, &device_settings))) {
                return controller::ControllerType_Unknown;
            }

            const controller::ControllerType type = controller::Identify(&device_settings);
            return IsSwitch2ControllerType(type) ? type : controller::ControllerType_Unknown;
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

        void SetBleConnection(const bluetooth::Address &address, u32 conn_id) {
            std::scoped_lock lk(g_conn_map_lock);
            g_ble_conn_ids[address] = conn_id;
        }

        void ClearBleConnection(const bluetooth::Address &address) {
            std::scoped_lock lk(g_conn_map_lock);
            g_ble_conn_ids.erase(address);
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
            {
                std::scoped_lock lk(g_switch2_connect_queue_lock);

                const u64 now_ns = GetCurrentTimeNs();
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

            if (!inserted) {
                return;
            }

            SW2_LOG_INFO(
                "Queued Switch 2 BLE connect request: reason=%s type=%u addr=%02X:%02X:%02X:%02X:%02X:%02X",
                reason,
                static_cast<u32>(type),
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
                        "Switch 2 connect worker: btmBleConnect type=%u addr=%02X:%02X:%02X:%02X:%02X:%02X",
                        static_cast<u32>(request.type),
                        request.address.address[0], request.address.address[1], request.address.address[2],
                        request.address.address[3], request.address.address[4], request.address.address[5]
                    );

                    const Result rc_connect = ConnectSwitch2Controller(request.address);
                    if (R_FAILED(rc_connect)) {
                        SW2_LOG_WARN("Switch 2 connect worker: btmBleConnect failed rc=0x%08X", static_cast<u32>(rc_connect.GetValue()));
                    }

                    os::SleepThread(ams::TimeSpan::FromMilliSeconds(100));
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
            g_switch2_cmd_responses[conn_id] = Switch2CommandResponseState{0, 0, 0, 0, 0};
            g_switch2_prefer_universal_input[conn_id] = false;
        }

        void ClearSwitch2CommandState(u32 conn_id) {
            std::scoped_lock lk(g_switch2_cmd_response_lock);
            g_switch2_cmd_responses.erase(conn_id);
            g_switch2_prefer_universal_input.erase(conn_id);
        }

        void SetSwitch2PreferUniversalInput(u32 conn_id, bool enabled) {
            std::scoped_lock lk(g_switch2_cmd_response_lock);
            g_switch2_prefer_universal_input[conn_id] = enabled;
        }

        bool GetSwitch2PreferUniversalInput(u32 conn_id) {
            std::scoped_lock lk(g_switch2_cmd_response_lock);
            auto it = g_switch2_prefer_universal_input.find(conn_id);
            return (it != g_switch2_prefer_universal_input.end()) ? it->second : false;
        }

        void RecordSwitch2CommandResponse(u32 conn_id, const u8 *data, u16 size) {
            if (size < 4) {
                return;
            }

            std::scoped_lock lk(g_switch2_cmd_response_lock);
            auto &state = g_switch2_cmd_responses[conn_id];
            state.sequence += 1;
            state.last_cmd = data[0];
            state.last_status = (size > 1) ? data[1] : 0;
            state.last_sub = data[3];
            state.last_ack = (size > 5) ? data[5] : 0;
        }

        bool WaitForSwitch2CommandResponse(u32 conn_id, u8 cmd_id, u8 sub_id, u32 timeout_ms) {
            u64 seen_sequence = 0;
            {
                std::scoped_lock lk(g_switch2_cmd_response_lock);
                auto it = g_switch2_cmd_responses.find(conn_id);
                if (it == g_switch2_cmd_responses.end()) {
                    return false;
                }
                seen_sequence = it->second.sequence;
            }

            const u64 timeout_ns = static_cast<u64>(timeout_ms) * 1'000'000ull;
            const u64 end_ns = GetCurrentTimeNs() + timeout_ns;

            while (GetCurrentTimeNs() < end_ns) {
                {
                    std::scoped_lock lk(g_switch2_cmd_response_lock);
                    auto it = g_switch2_cmd_responses.find(conn_id);
                    if (it == g_switch2_cmd_responses.end()) {
                        return false;
                    }

                    if (it->second.sequence != seen_sequence) {
                        seen_sequence = it->second.sequence;
                        if (it->second.last_cmd == cmd_id && it->second.last_sub == sub_id) {
                            return true;
                        }
                    }
                }

                os::SleepThread(ams::TimeSpan::FromMilliSeconds(5));
            }

            return false;
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

        Result RegisterSwitch2GattNotification(u32 conn_id, const BtdrvGattAttributeUuid &char_uuid) {
            const BtdrvGattId service_id = MakeGattId(Switch2InputServiceUuid);
            const BtdrvGattId char_id = MakeGattId(char_uuid);
            R_RETURN(btdrvRegisterGattNotification(conn_id, true, &service_id, &char_id));
        }

        Result RegisterSwitch2GattNotifications(u32 conn_id, controller::ControllerType type) {
            const auto input_uuid = GetSwitch2InputCharUuid(type);
            if (!input_uuid) {
                return MAKERESULT(0x123, 2);
            }

            R_TRY(RegisterSwitch2GattNotification(conn_id, Switch2CommandResponse1CharUuid));
            R_TRY(RegisterSwitch2GattNotification(conn_id, Switch2InputReport0x05CharUuid));
            R_TRY(RegisterSwitch2GattNotification(conn_id, *input_uuid));

            R_SUCCEED();
        }

        Result WriteSwitch2GattDataReportImpl(u32 conn_id, const bluetooth::HidReport *report) {
            const BtdrvGattId service_id = MakeGattId(Switch2InputServiceUuid);
            const BtdrvGattId command_id = MakeGattId(Switch2CommandCharUuid);
            return btdrvWriteGattCharacteristic(conn_id, true, &service_id, &command_id, report->data, report->size, 0, false);
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

        void LogBleEventSummary(const bluetooth::BleEventType type, const bluetooth::BleEventInfo &info) {
            switch (type) {
                case BtdrvBleEventType_ClientRegistration:
                    SW2_LOG_INFO(
                        "BLE ClientRegistration: result=%u client_if=%u status=%u",
                        info.client_registration.result,
                        info.client_registration.client_if,
                        info.client_registration.status
                    );
                    break;
                case BtdrvBleEventType_ScanFilter:
                    SW2_LOG_INFO(
                        "BLE ScanFilter: result=%u action=%u",
                        info.scan_filter.result,
                        info.scan_filter.action
                    );
                    break;
                case BtdrvBleEventType_ScanResult:
                    SW2_LOG_INFO(
                        "BLE ScanResult: result=%u status=%u(%s) count=%u addr=%02X:%02X:%02X:%02X:%02X:%02X rssi=%d",
                        info.scan_result.result,
                        info.scan_result.status,
                        GetScanStatusName(info.scan_result.status),
                        info.scan_result.count,
                        info.scan_result.address.address[0], info.scan_result.address.address[1], info.scan_result.address.address[2],
                        info.scan_result.address.address[3], info.scan_result.address.address[4], info.scan_result.address.address[5],
                        info.scan_result.rssi
                    );
                    break;
                case BtdrvBleEventType_ClientConnection:
                    SW2_LOG_INFO(
                        "BLE ClientConnection: result=%u status=%u conn_id=%u addr=%02X:%02X:%02X:%02X:%02X:%02X reason=%u",
                        info.client_connection.result,
                        info.client_connection.status,
                        info.client_connection.conn_id,
                        info.client_connection.address.address[0], info.client_connection.address.address[1], info.client_connection.address.address[2],
                        info.client_connection.address.address[3], info.client_connection.address.address[4], info.client_connection.address.address[5],
                        info.client_connection.reason
                    );
                    break;
                case BtdrvBleEventType_ClientNotify:
                {
                    char serv_uuid[64] = {};
                    char char_uuid[64] = {};
                    FormatGattUuid(serv_uuid, sizeof(serv_uuid), info.client_notify.serv_uuid);
                    FormatGattUuid(char_uuid, sizeof(char_uuid), info.client_notify.char_uuid);

                    SW2_LOG_INFO(
                        "BLE ClientNotify: result=%u conn_id=%u size=%u type=%u serv=%s char=%s",
                        info.client_notify.result,
                        info.client_notify.conn_id,
                        info.client_notify.size,
                        info.client_notify.type,
                        serv_uuid,
                        char_uuid
                    );
                    break;
                }
                default:
                    break;
            }
        }

        void ScanForSwitch2(const bluetooth::BleEventInfo &info) {
            const u8 *data = reinterpret_cast<const u8 *>(&info);
            const size_t size = sizeof(bluetooth::BleEventInfo);

            if (size < sizeof(Switch2ManufacturerPrefix)) {
                return;
            }

            for (size_t i = 0; i <= size - sizeof(Switch2ManufacturerPrefix); ++i) {
                if (std::memcmp(&data[i], Switch2ManufacturerPrefix, sizeof(Switch2ManufacturerPrefix)) == 0) {
                    SW2_LOG_INFO("Switch 2 manufacturer pattern observed in BLE event at offset %zu", i);
                    break;
                }
            }
        }

        void HandleScanResultEvent(const bluetooth::BleEventInfo &info) {
            const auto &sr = info.scan_result;

            // Passive observation only. btm-sysmodule owns the scan lifecycle; we never
            // restart it or initiate connections from this callback (that destabilises
            // the system). We log every result so we can confirm whether the console's
            // own scan ever surfaces a Switch 2 advertisement to this MITM.
            //
            // status: 0xFF=scan started, 2=new device found, 1=scan complete.
            if (sr.status != 2 || sr.count == 0) {
                return;
            }

            SW2_LOG_INFO(
                "ScanResult device: addr=%02X:%02X:%02X:%02X:%02X:%02X type=%u addr_type=%u rssi=%d ads=%u",
                sr.address.address[0], sr.address.address[1], sr.address.address[2],
                sr.address.address[3], sr.address.address[4], sr.address.address[5],
                static_cast<u32>(sr.device_type), static_cast<u32>(sr.ble_addr_type),
                sr.rssi, static_cast<u32>(sr.count)
            );

            controller::ControllerType detected = controller::ControllerType_Unknown;

            for (size_t i = 0; i < sr.count && i < std::size(sr.ad_list); ++i) {
                const auto &ad = sr.ad_list[i];
                if (ad.size == 0 || ad.size > sizeof(ad.data)) {
                    continue;
                }

                // Dump each advertisement structure so we can verify the on-air format
                // against documentation/bluetooth_interface.md regardless of a match.
                char hex[3 * sizeof(ad.data) + 1];
                size_t pos = 0;
                for (size_t b = 0; b < ad.size && pos + 3 < sizeof(hex); ++b) {
                    pos += std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", ad.data[b]);
                }
                SW2_LOG_VERBOSE("  AD type=0x%02X size=%u data=%s", ad.type, static_cast<u32>(ad.size), hex);

                if (MatchSwitch2ManufacturerData(ad)) {
                    SW2_LOG_INFO("Switch 2 manufacturer AD detected in scan result (%u bytes)", static_cast<u32>(ad.size));

                    controller::ControllerType t = DecodeSwitch2TypeFromManufacturerData(ad.data, ad.size);
                    if (t != controller::ControllerType_Unknown) {
                        detected = t;
                        break;
                    }
                }
            }

            if (detected != controller::ControllerType_Unknown) {
                controller::RegisterDiscoveredSwitch2Controller(sr.address, detected);
                SW2_LOG_INFO(
                    "Registered Switch 2 candidate from scan: type=%u addr=%02X:%02X:%02X:%02X:%02X:%02X rssi=%d",
                    static_cast<u32>(detected),
                    sr.address.address[0], sr.address.address[1], sr.address.address[2],
                    sr.address.address[3], sr.address.address[4], sr.address.address[5],
                    sr.rssi
                );
                QueueSwitch2ConnectRequest(sr.address, detected, "ScanResult");
            }
        }

        void HandleClientConnectionEvent(const bluetooth::BleEventInfo &info) {
            const auto &cc = info.client_connection;

            if (cc.status == 0) {
                {
                    std::scoped_lock lk(g_conn_map_lock);
                    g_conn_map[cc.conn_id] = cc.address;
                }
                SetBleConnection(cc.address, cc.conn_id);
                ResetSwitch2CommandState(cc.conn_id);

                SW2_LOG_INFO(
                    "BLE client connected: conn_id=%u addr=%02X:%02X:%02X:%02X:%02X:%02X",
                    cc.conn_id,
                    cc.address.address[0], cc.address.address[1], cc.address.address[2],
                    cc.address.address[3], cc.address.address[4], cc.address.address[5]
                );

                controller::ControllerType sw2_type = controller::GetDiscoveredSwitch2ControllerType(cc.address);
                if (sw2_type == controller::ControllerType_Unknown) {
                    sw2_type = ResolveSwitch2TypeFromPairedInfo(cc.address);
                    if (sw2_type != controller::ControllerType_Unknown) {
                        controller::RegisterDiscoveredSwitch2Controller(cc.address, sw2_type);
                        SW2_LOG_INFO("Resolved Switch 2 type from paired info: type=%u", static_cast<u32>(sw2_type));
                    }
                }

                if (sw2_type != controller::ControllerType_Unknown) {
                    const Result rc_notify = RegisterSwitch2GattNotifications(cc.conn_id, sw2_type);
                    if (R_FAILED(rc_notify)) {
                        SW2_LOG_WARN("Failed to register Switch 2 BLE notifications: 0x%08X", static_cast<u32>(rc_notify.GetValue()));
                    }

                    if (!controller::LocateHandler(cc.address)) {
                        controller::AttachHandler(cc.address);
                    }
                } else {
                    SW2_LOG_WARN("BLE client connected without Switch 2 discovery type; skipping Switch 2 notification registration");
                }
            } else if (cc.status == 2) {
                {
                    std::scoped_lock lk(g_conn_map_lock);
                    g_conn_map.erase(cc.conn_id);
                }
                ClearBleConnection(cc.address);
                ClearSwitch2CommandState(cc.conn_id);

                SW2_LOG_INFO(
                    "BLE client disconnected: conn_id=%u addr=%02X:%02X:%02X:%02X:%02X:%02X reason=%u",
                    cc.conn_id,
                    cc.address.address[0], cc.address.address[1], cc.address.address[2],
                    cc.address.address[3], cc.address.address[4], cc.address.address[5],
                    cc.reason
                );

                controller::RemoveHandler(cc.address);
            }
        }

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
                }
            }

            if (!have_addr) {
                SW2_LOG_WARN("BLE notify without known conn_id mapping: conn_id=%u size=%u", cn.conn_id, cn.size);
                return;
            }

            if (!UuidEquals(cn.serv_uuid, Switch2InputServiceUuid)) {
                SW2_LOG_VERBOSE("Ignoring BLE notify for non-Switch2 service: conn_id=%u size=%u", cn.conn_id, cn.size);
                return;
            }

            if (UuidEquals(cn.char_uuid, Switch2CommandResponse1CharUuid)) {
                RecordSwitch2CommandResponse(cn.conn_id, cn.data, cn.size);
                if (cn.size >= 6) {
                    SW2_LOG_VERBOSE(
                        "Switch 2 command response: conn_id=%u cmd=0x%02X status=0x%02X sub=0x%02X ack=0x%02X",
                        cn.conn_id,
                        cn.data[0],
                        cn.data[1],
                        cn.data[3],
                        cn.data[5]
                    );
                }
                return;
            }

            const bool is_universal_report = UuidEquals(cn.char_uuid, Switch2InputReport0x05CharUuid);
            if (is_universal_report) {
                SetSwitch2PreferUniversalInput(cn.conn_id, true);
            }

            const bool prefer_universal = GetSwitch2PreferUniversalInput(cn.conn_id);
            if (prefer_universal && !is_universal_report) {
                SW2_LOG_VERBOSE("Ignoring type-specific BLE notify while universal 0x05 stream is active: conn_id=%u", cn.conn_id);
                return;
            }

            controller::ControllerType notify_type = controller::ControllerType_Unknown;
            if (!is_universal_report) {
                notify_type = DecodeSwitch2TypeFromInputUuid(cn.char_uuid);
            }

            if (!is_universal_report && notify_type == controller::ControllerType_Unknown) {
                SW2_LOG_VERBOSE("Ignoring BLE notify for non-input Switch2 characteristic: conn_id=%u size=%u", cn.conn_id, cn.size);
                return;
            }

            controller::ControllerType known_type = controller::GetDiscoveredSwitch2ControllerType(addr);
            if (known_type == controller::ControllerType_Unknown) {
                if (notify_type != controller::ControllerType_Unknown) {
                    known_type = notify_type;
                    controller::RegisterDiscoveredSwitch2Controller(addr, known_type);
                    SW2_LOG_INFO("Inferred Switch 2 controller type from input characteristic: type=%u", static_cast<u32>(known_type));
                } else {
                    known_type = ResolveSwitch2TypeFromPairedInfo(addr);
                    if (known_type != controller::ControllerType_Unknown) {
                        controller::RegisterDiscoveredSwitch2Controller(addr, known_type);
                        SW2_LOG_INFO("Resolved Switch 2 controller type from paired info for universal report stream: type=%u", static_cast<u32>(known_type));
                    }
                }
            } else if (known_type != notify_type) {
                if (notify_type != controller::ControllerType_Unknown) {
                    SW2_LOG_WARN("Switch 2 notify type mismatch: discovered=%u notify=%u", static_cast<u32>(known_type), static_cast<u32>(notify_type));
                }
            }

            if (known_type == controller::ControllerType_Unknown) {
                SW2_LOG_WARN("Switch 2 input notify dropped: unresolved controller type conn_id=%u size=%u", cn.conn_id, cn.size);
                return;
            }

            auto device = controller::LocateHandler(addr);
            if (!device) {
                controller::AttachHandler(addr);
                device = controller::LocateHandler(addr);
            }

            if (!device) {
                SW2_LOG_WARN("No controller handler for BLE notify: conn_id=%u", cn.conn_id);
                return;
            }

            bluetooth::HidReport report = {};
            const u16 copy_size = std::min<u16>(cn.size, sizeof(report.data));
            report.size = copy_size;
            std::memcpy(report.data, cn.data, copy_size);

            bluetooth::HidReportEventInfo event_info = {};
            event_info.data_report.v9.res = 0;
            event_info.data_report.v9.proto_mode = 0;
            event_info.data_report.v9.addr = addr;
            std::memcpy(&event_info.data_report.v9.report, &report, sizeof(report));

            SW2_LOG_VERBOSE("BLE notify routed to controller: conn_id=%u size=%u", cn.conn_id, copy_size);
            if (R_FAILED(device->HandleDataReportEvent(&event_info))) {
                SW2_LOG_WARN("HandleDataReportEvent failed for BLE notify: conn_id=%u", cn.conn_id);
            }
        }

    }

    bool IsInitialized() {
        return g_init_event.TryWait();
    }

    void SignalInitialized() {
        g_init_event.Signal();
        StartSwitch2ConnectWorkerIfNeeded();
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
        std::memcpy(buffer, &g_event_info, size);

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
        u32 conn_id = 0;
        if (!HasBleConnection(address, &conn_id)) {
            SW2_LOG_WARN("WriteSwitch2GattDataReport: missing BLE connection mapping (size=%u)", report->size);
            return MAKERESULT(0x123, 1);
        }

        const Result rc = WriteSwitch2GattDataReportImpl(conn_id, report);
        if (R_FAILED(rc)) {
            SW2_LOG_WARN("WriteSwitch2GattDataReport failed: conn_id=%u rc=0x%08X size=%u", conn_id, static_cast<u32>(rc.GetValue()), report->size);
        }

        if (R_SUCCEEDED(rc) && ShouldWaitForSwitch2CommandResponse(report)) {
            const bool got_response = WaitForSwitch2CommandResponse(conn_id, report->data[0], report->data[3], 300);
            if (!got_response) {
                SW2_LOG_WARN(
                    "Switch 2 command response timeout: conn_id=%u cmd=0x%02X sub=0x%02X",
                    conn_id,
                    report->data[0],
                    report->data[3]
                );
            }
        }

        return rc;
    }

    void HandleEvent() {
        {
            std::scoped_lock lk(g_event_data_lock);
            R_ABORT_UNLESS(btdrvGetBleManagedEventInfo(&g_event_info, sizeof(bluetooth::BleEventInfo), &g_current_event_type));
        }

        SW2_LOG_INFO("BLE Event: type=%u (%s)", (u32)g_current_event_type, GetBleEventTypeName(g_current_event_type));
        LogBleEventSummary(g_current_event_type, g_event_info);
        
        // Log raw data (at most 64 bytes)
        u8 *raw = reinterpret_cast<u8*>(&g_event_info);
        char hex[256];
        size_t pos = 0;
        for (size_t i = 0; i < sizeof(bluetooth::BleEventInfo) && i < 64 && pos + 3 < sizeof(hex); ++i) {
            pos += std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", raw[i]);
        }
        SW2_LOG_VERBOSE("BLE Data: %s", hex);

        // Switch 2 controllers communicate over BLE GATT. This handler currently only
        // observes/forwards managed BLE events and logs them for debugging. The full
        // integration point for Switch 2 support lives here: a BLE GATT bridge needs to
        // (1) detect Switch 2 advertisements, (2) connect as GATT central, (3) discover
        // services, (4) subscribe to the controller-specific default input
        // characteristic (0x07/0x08/0x09/0x0A on handle 0x000E), and (5) on each
        // notification build a bluetooth::HidReport from the raw payload and route it
        // to the matching controller's ProcessInputData(). Outbound init/LED commands
        // built by Switch2Controller::MakeSwitch2Command() must be written to the
        // command characteristic (649d4ac9-8eb7-4e6c-af44-1ea54fe5f005).
        ScanForSwitch2(g_event_info);

        switch (g_current_event_type) {
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

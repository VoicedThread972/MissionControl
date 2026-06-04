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
#include "bluetooth_hid.hpp"
#include "../btdrv_mitm_flags.hpp"
#include "../../controllers/controller_management.hpp"
#include "../../controllers/switch2_debug.hpp"
#include "../../utils.hpp"

namespace ams::bluetooth::hid {

    namespace {

        constinit os::SdkMutex g_event_info_lock;
        constinit bluetooth::HidEventInfo g_event_info;
        constinit bluetooth::HidEventType g_current_event_type;

        os::SystemEvent g_system_event;
        os::SystemEvent g_system_event_fwd(os::EventClearMode_AutoClear, true);
        os::SystemEvent g_system_event_user_fwd(os::EventClearMode_AutoClear, true);

        os::Event g_init_event(os::EventClearMode_ManualClear);
        os::Event g_data_read_event(os::EventClearMode_AutoClear);

        void FormatAddress(char *buf, size_t bufsz, const bluetooth::Address &addr) {
            util::SNPrintf(
                buf,
                bufsz,
                "%02X:%02X:%02X:%02X:%02X:%02X",
                addr.address[0], addr.address[1], addr.address[2],
                addr.address[3], addr.address[4], addr.address[5]
            );
        }

        const char *GetHidEventTypeName(const bluetooth::HidEventType type) {
            switch (type) {
                case BtdrvHidEventType_Connection:
                    return "Connection";
                case BtdrvHidEventType_Data:
                case BtdrvHidEventTypeOld_Data:
                    return "Data";
                case BtdrvHidEventType_SetReport:
                case BtdrvHidEventTypeOld_SetReport:
                    return "SetReport";
                case BtdrvHidEventType_GetReport:
                case BtdrvHidEventTypeOld_GetReport:
                    return "GetReport";
                case BtdrvHidEventTypeOld_Ext:
                    return "Ext";
                default:
                    return "Unknown";
            }
        }

    }

    bool IsInitialized() {
        return g_init_event.TryWait();
    }

    void SignalInitialized() {
        g_init_event.Signal();
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

    void SignalFakeEvent(bluetooth::HidEventType type, const void *data, size_t size) {
        g_current_event_type = type;
        std::memcpy(&g_event_info, data, size);

        g_system_event_fwd.Signal();
    }

    Result GetEventInfo(bluetooth::HidEventType *type, void *buffer, size_t size) {
        std::scoped_lock lk(g_event_info_lock);

        *type = g_current_event_type;
        std::memcpy(buffer, &g_event_info, size);

        g_data_read_event.Signal();
        
        R_SUCCEED();
    }

    inline void HandleConnectionStateEventV1(bluetooth::HidEventInfo *event_info) {
        char addr_str[20];
        FormatAddress(addr_str, sizeof(addr_str), event_info->connection.v1.addr);

        switch (event_info->connection.v1.status) {
            case BtdrvHidConnectionStatusOld_Opened:
                SW2_LOG_INFO("HID Connection (v1): opened addr=%s", addr_str);
                controller::AttachHandler(event_info->connection.v1.addr);
                break;
            case BtdrvHidConnectionStatusOld_Closed:
                SW2_LOG_INFO("HID Connection (v1): closed addr=%s", addr_str);
                controller::RemoveHandler(event_info->connection.v1.addr);
                break;
            case BtdrvHidConnectionStatusOld_Failed:
                SW2_LOG_WARN("HID Connection (v1): failed addr=%s", addr_str);
                break;
            default:
                SW2_LOG_VERBOSE("HID Connection (v1): status=%u addr=%s", event_info->connection.v1.status, addr_str);
                break;
        }
    }

    inline void HandleConnectionStateEventV12(bluetooth::HidEventInfo *event_info) {
        char addr_str[20];
        FormatAddress(addr_str, sizeof(addr_str), event_info->connection.v12.addr);

        switch (event_info->connection.v12.status) {
            case BtdrvHidConnectionStatus_Opened:
                SW2_LOG_INFO("HID Connection: opened addr=%s", addr_str);
                controller::AttachHandler(event_info->connection.v12.addr);
                break;
            case BtdrvHidConnectionStatus_Closed:
                SW2_LOG_INFO("HID Connection: closed addr=%s", addr_str);
                controller::RemoveHandler(event_info->connection.v12.addr);
                break;
            case BtdrvHidConnectionStatus_Failed:
                SW2_LOG_WARN("HID Connection: failed addr=%s", addr_str);
                break;
            default:
                SW2_LOG_VERBOSE("HID Connection: status=%u addr=%s", event_info->connection.v12.status, addr_str);
                break;
        }
    }

    void HandleEvent() {
        {
            std::scoped_lock lk(g_event_info_lock);
            R_ABORT_UNLESS(btdrvGetHidEventInfo(&g_event_info, sizeof(bluetooth::HidEventInfo), &g_current_event_type));
        }

        SW2_LOG_INFO("HID Event: type=%u (%s)", (u32)g_current_event_type, GetHidEventTypeName(g_current_event_type));

        switch (g_current_event_type) {
            case BtdrvHidEventType_Connection:
                hos::GetVersion() < hos::Version_12_0_0 ? HandleConnectionStateEventV1(&g_event_info) : HandleConnectionStateEventV12(&g_event_info);
                break;
            default:
                break;
        }

        g_system_event_fwd.Signal();
        g_data_read_event.Wait();

        if (g_system_event_user_fwd.GetBase()->state) {
            g_system_event_user_fwd.Signal();
        }
    }

}

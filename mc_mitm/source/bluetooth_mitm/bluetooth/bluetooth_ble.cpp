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

        // Canonical Switch 2 manufacturer-specific advertising prefix, taken from the
        // joycon2cpp reference (company id 0x0001 followed by 0x03 0x7E). This is used
        // purely as a debugging aid to confirm that Switch 2 controllers are visible to
        // the BLE stack. It deliberately does NOT fabricate a device address or register
        // a controller type: the managed BLE event buffer is not an advertising report,
        // and the actual controller attach must be driven by the BLE GATT bridge (see
        // the integration notes in HandleEvent below) once that transport is wired up.
        constexpr u8 Switch2ManufacturerPrefix[] = { 0x01, 0x00, 0x03, 0x7E };

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

    Result GetEventInfo(bluetooth::BleEventType *type, void *buffer, size_t size) {
        std::scoped_lock lk(g_event_data_lock);

        *type = g_current_event_type;
        std::memcpy(buffer, &g_event_info, size);

        g_data_read_event.Signal();
        
        R_SUCCEED();
    }

    void HandleEvent() {
        {
            std::scoped_lock lk(g_event_data_lock);
            R_ABORT_UNLESS(btdrvGetBleManagedEventInfo(&g_event_info, sizeof(bluetooth::BleEventInfo), &g_current_event_type));
        }

        SW2_LOG_INFO("BLE Event: type=%u", (u32)g_current_event_type);
        
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
        // services, (4) subscribe to the 0x05 input characteristic
        // (ab7de9be-89fe-49ad-828f-118f09df7fd2), and (5) on each notification build a
        // bluetooth::HidReport from the raw payload and route it to the matching
        // controller's ProcessInputData(). Outbound init/LED commands built by
        // Switch2Controller::MakeSwitch2Command() must be written to the command
        // characteristic (649d4ac9-8eb7-4e6c-af44-1ea54fe5f005).
        ScanForSwitch2(g_event_info);

        if (!g_redirect_ble_events) {
            g_system_event_fwd.Signal();
            g_data_read_event.Wait();
        }

        if (g_system_event_user_fwd.GetBase()->state) {
            g_system_event_user_fwd.Signal();
        }
    }

}

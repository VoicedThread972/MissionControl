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
#include "emulated_switch_controller.hpp"

namespace ams::controller {

    /*
     * All Switch 2 controllers (Joy-Con 2 L/R, Pro Controller 2, NSO GameCube)
     * expose a single universal input report, id 0x05, over their BLE input
     * characteristic (UUID ab7de9be-89fe-49ad-828f-118f09df7fd2). When delivered
     * over BLE the leading report-id byte is omitted, so byte 0 of the payload is
     * the first field of the report (a free-running counter).
     *
     * This layout, the button bit ordering and the 12-bit stick packing are taken
     * directly from the canonical joycon2cpp reference implementation and confirmed
     * against documentation/hid_reports.md.
     */

    // Button bit layout of report 0x05 (4 bytes, starting at payload offset 0x04).
    // The first three bytes are intentionally bit-compatible with SwitchButtonData.
    struct Switch2ButtonData {
        // byte 0 (offset 0x04)
        u8 Y            : 1;
        u8 X            : 1;
        u8 B            : 1;
        u8 A            : 1;
        u8 SR_right     : 1;
        u8 SL_right     : 1;
        u8 R            : 1;
        u8 ZR           : 1;

        // byte 1 (offset 0x05)
        u8 minus        : 1;
        u8 plus         : 1;
        u8 rstick_press : 1;
        u8 lstick_press : 1;
        u8 home         : 1;
        u8 capture      : 1;
        u8 C            : 1;
        u8              : 1;

        // byte 2 (offset 0x06)
        u8 dpad_down    : 1;
        u8 dpad_up      : 1;
        u8 dpad_right   : 1;
        u8 dpad_left    : 1;
        u8 SR_left      : 1;
        u8 SL_left      : 1;
        u8 L            : 1;
        u8 ZL           : 1;

        // byte 3 (offset 0x07)
        u8 GR           : 1;
        u8 GL           : 1;
        u8              : 2;
        u8 headset      : 1;
        u8              : 3;
    } PACKED;

    struct Switch2InputReport0x05 {
        u8                counter[4];      // 0x00 free-running packet counter
        Switch2ButtonData buttons;         // 0x04 digital buttons
        u8                unk0x08[2];       // 0x08
        u8                left_stick[3];    // 0x0A 12-bit packed (x, y)
        u8                right_stick[3];   // 0x0D 12-bit packed (x, y)
    } PACKED;

    // Absolute payload offsets for fields not covered by the leading struct.
    enum Switch2InputOffset {
        Switch2InputOffset_BatteryVoltage = 0x1F,   // u16, little-endian, millivolts
        Switch2InputOffset_TriggerL       = 0x3C,   // u8, analog (GameCube controller only)
        Switch2InputOffset_TriggerR       = 0x3D,   // u8, analog (GameCube controller only)
    };

    // Minimum payload length required to safely read the stick fields.
    constexpr size_t Switch2InputReportMinLength = sizeof(Switch2InputReport0x05);

    // Analog-trigger threshold above which a GameCube ZL/ZR is treated as pressed.
    constexpr u8 Switch2TriggerThreshold = 30;

    class Switch2Controller : public EmulatedSwitchController {
        public:
            Switch2Controller(bluetooth::Address address, HardwareID id)
            : EmulatedSwitchController(address, id) { }

            Result Initialize() override;

            void ProcessInputData(const bluetooth::HidReport *report) override;

        protected:
            // Builds a Switch 2 command frame to be written to the controller's
            // command characteristic (UUID 649d4ac9-8eb7-4e6c-af44-1ea54fe5f005).
            void MakeSwitch2Command(bluetooth::HidReport *report, u8 cmd_id, u8 sub_id, const u8 *data, u8 data_len);

            // Maps the universal report 0x05 button/stick/battery state onto the
            // emulated Switch controller. Shared by every Switch 2 controller type.
            void MapInputReport0x05(const bluetooth::HidReport *report);

            static u16 Unpack12BitStickX(const u8 *data) {
                return static_cast<u16>(((data[1] & 0x0F) << 8) | data[0]);
            }
            static u16 Unpack12BitStickY(const u8 *data) {
                return static_cast<u16>((data[2] << 4) | ((data[1] & 0xF0) >> 4));
            }
    };

    class JoyCon2LController final : public Switch2Controller {
        public:
            static constexpr const HardwareID hardware_ids[] = {
                {0x057e, 0x2060}
            };

            JoyCon2LController(bluetooth::Address address, HardwareID id)
            : Switch2Controller(address, id) { }
    };

    class JoyCon2RController final : public Switch2Controller {
        public:
            static constexpr const HardwareID hardware_ids[] = {
                {0x057e, 0x2061}
            };

            JoyCon2RController(bluetooth::Address address, HardwareID id)
            : Switch2Controller(address, id) { }
    };

    class ProController2Controller final : public Switch2Controller {
        public:
            static constexpr const HardwareID hardware_ids[] = {
                {0x057e, 0x2062}
            };

            ProController2Controller(bluetooth::Address address, HardwareID id)
            : Switch2Controller(address, id) { }
    };

    class NSOGCController2Controller final : public Switch2Controller {
        public:
            static constexpr const HardwareID hardware_ids[] = {
                {0x057e, 0x2064}
            };

            NSOGCController2Controller(bluetooth::Address address, HardwareID id)
            : Switch2Controller(address, id) { }

            // The GameCube controller reports ZL/ZR as analog triggers rather than
            // digital buttons, so it extends the shared mapping.
            void ProcessInputData(const bluetooth::HidReport *report) override;
    };

}

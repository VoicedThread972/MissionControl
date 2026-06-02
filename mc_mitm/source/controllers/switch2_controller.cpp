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
#include "switch2_controller.hpp"
#include "controller_utils.hpp"
#include "switch2_debug.hpp"
#include <stratosphere.hpp>

namespace ams::controller {

    namespace {

        const u8 FeatureMask[] = { 0xFF, 0x00, 0x00, 0x00 };

        // Maps a Switch 2 battery voltage (millivolts) onto the 0-8 scale used by
        // the emulated Switch controller.
        u8 convert_battery_voltage(u16 voltage_mv) {
            if (voltage_mv >= 4000) return 8;
            if (voltage_mv >= 3700) return 6;
            if (voltage_mv >= 3500) return 4;
            if (voltage_mv >= 3300) return 2;
            return 0;
        }

    }

    // --- Switch2Controller base ---

    void Switch2Controller::MakeSwitch2Command(bluetooth::HidReport *report, u8 cmd_id, u8 sub_id, const u8 *data, u8 data_len) {
        report->data[0] = cmd_id;
        report->data[1] = 0x91;
        report->data[2] = 0x01;
        report->data[3] = sub_id;
        report->data[4] = 0x00;
        report->data[5] = data_len;
        report->data[6] = 0x00;
        report->data[7] = 0x00;
        if (data && data_len > 0) {
            std::memcpy(&report->data[8], data, data_len);
        }
        report->size = 8 + data_len;
    }

    Result Switch2Controller::Initialize() {
        R_TRY(this->EmulatedSwitchController::Initialize());

        SW2_LOG_INFO("Initializing Switch 2 controller (vid=0x%04x, pid=0x%04x)", m_id.vid, m_id.pid);

        bluetooth::HidReport report;
        
        // 1. Set Feature Mask
        this->MakeSwitch2Command(&report, 0x0C, 0x02, FeatureMask, sizeof(FeatureMask));
        R_TRY(this->WriteDataReport(&report));
        os::SleepThread(ams::TimeSpan::FromMilliSeconds(50));

        // 2. Enable Features
        this->MakeSwitch2Command(&report, 0x0C, 0x04, FeatureMask, sizeof(FeatureMask));
        R_TRY(this->WriteDataReport(&report));
        os::SleepThread(ams::TimeSpan::FromMilliSeconds(50));

        // 3. Set Player LEDs. The reference sends a single LED bitmask byte; bit 0
        // lit corresponds to player 1.
        const u8 led_data[] = { 0x01 };
        this->MakeSwitch2Command(&report, 0x09, 0x07, led_data, sizeof(led_data));
        R_TRY(this->WriteDataReport(&report));

        R_SUCCEED();
    }

    void Switch2Controller::ProcessInputData(const bluetooth::HidReport *report) {
        this->MapInputReport0x05(report);
    }

    void Switch2Controller::MapInputReport0x05(const bluetooth::HidReport *report) {
        // Every Switch 2 controller delivers the same universal report 0x05 over
        // its BLE input characteristic. Guard against short/garbage packets.
        if (report->size < Switch2InputReportMinLength) {
            return;
        }

        auto src = reinterpret_cast<const Switch2InputReport0x05 *>(report->data);
        const auto &btn = src->buttons;

        // The first three button bytes of report 0x05 share their bit ordering with
        // SwitchButtonData, so the common fields map across directly. Buttons that
        // are not physically present on a given controller simply read as zero.
        m_buttons.A = btn.A;
        m_buttons.B = btn.B;
        m_buttons.X = btn.X;
        m_buttons.Y = btn.Y;

        m_buttons.dpad_down  = btn.dpad_down;
        m_buttons.dpad_up    = btn.dpad_up;
        m_buttons.dpad_right = btn.dpad_right;
        m_buttons.dpad_left  = btn.dpad_left;

        m_buttons.L  = btn.L;
        m_buttons.R  = btn.R;
        m_buttons.ZL = btn.ZL;
        m_buttons.ZR = btn.ZR;

        m_buttons.minus        = btn.minus;
        m_buttons.plus         = btn.plus;
        m_buttons.lstick_press = btn.lstick_press;
        m_buttons.rstick_press = btn.rstick_press;
        m_buttons.home         = btn.home;
        m_buttons.capture      = btn.capture;

        m_left_stick.SetData(
            Unpack12BitStickX(src->left_stick),
            Unpack12BitStickY(src->left_stick)
        );
        m_right_stick.SetData(
            Unpack12BitStickX(src->right_stick),
            Unpack12BitStickY(src->right_stick)
        );

        if (report->size > Switch2InputOffset_BatteryVoltage + 1) {
            const u16 voltage_mv = static_cast<u16>(report->data[Switch2InputOffset_BatteryVoltage] |
                                                   (report->data[Switch2InputOffset_BatteryVoltage + 1] << 8));
            m_battery = convert_battery_voltage(voltage_mv);
        }
    }

    // --- NSO GameCube Controller 2 ---

    void NSOGCController2Controller::ProcessInputData(const bluetooth::HidReport *report) {
        // Reuse the shared button/stick/battery mapping, then override ZL/ZR using
        // the GameCube controller's analog trigger axes.
        this->MapInputReport0x05(report);

        if (report->size > Switch2InputOffset_TriggerR) {
            m_buttons.ZL = report->data[Switch2InputOffset_TriggerL] > Switch2TriggerThreshold ? 1 : 0;
            m_buttons.ZR = report->data[Switch2InputOffset_TriggerR] > Switch2TriggerThreshold ? 1 : 0;
        }
    }

}

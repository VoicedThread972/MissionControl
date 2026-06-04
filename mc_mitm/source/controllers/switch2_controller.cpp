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

        constexpr u8 FeatureMaskJoyCon2 = 0x37;
        constexpr u8 FeatureMaskPro2    = 0x2F;
        constexpr u8 FeatureMaskNsoGc   = 0x27;

        // Maps a Switch 2 battery voltage (millivolts) onto the 0-8 scale used by
        // the emulated Switch controller.
        u8 convert_battery_voltage(u16 voltage_mv) {
            if (voltage_mv >= 4000) return 8;
            if (voltage_mv >= 3700) return 6;
            if (voltage_mv >= 3500) return 4;
            if (voltage_mv >= 3300) return 2;
            return 0;
        }

        u8 convert_battery_level(u8 level) {
            level &= 0x0F;
            if (level == 0) return 0;
            if (level <= 2) return 2;
            if (level <= 4) return 4;
            if (level <= 6) return 6;
            return 8;
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

    u8 Switch2Controller::GetFeatureMask() const {
        switch (m_id.pid) {
            case 0x2060:
            case 0x2061:
                return FeatureMaskJoyCon2;
            case 0x2062:
                return FeatureMaskPro2;
            case 0x2064:
                return FeatureMaskNsoGc;
            default:
                return FeatureMaskPro2;
        }
    }

    void Switch2Controller::ResetSwitch2StateForReport() {
        std::memset(&m_buttons, 0, sizeof(m_buttons));
        m_left_stick.SetData(SwitchAnalogStick::Center, SwitchAnalogStick::Center);
        m_right_stick.SetData(SwitchAnalogStick::Center, SwitchAnalogStick::Center);
    }

    void Switch2Controller::ApplyPowerInfo(u8 power_info) {
        m_ext_power = (power_info & 0x01) != 0;
        m_charging = (power_info & 0x02) != 0;
        m_battery = convert_battery_level((power_info >> 2) & 0x0F);
    }

    Result Switch2Controller::Initialize() {
        R_TRY(this->EmulatedSwitchController::Initialize());

        SW2_LOG_INFO("Initializing Switch 2 controller (vid=0x%04x, pid=0x%04x)", m_id.vid, m_id.pid);

        bluetooth::HidReport report;

        // Observed early bootstrap commands from the captured Bluetooth sequence.
        this->MakeSwitch2Command(&report, 0x07, 0x01, nullptr, 0);
        R_TRY(this->WriteDataReport(&report));
        os::SleepThread(ams::TimeSpan::FromMilliSeconds(20));

        if (m_id.pid == 0x2060 || m_id.pid == 0x2061 || m_id.pid == 0x2064) {
            this->MakeSwitch2Command(&report, 0x10, 0x01, nullptr, 0);
            R_TRY(this->WriteDataReport(&report));
            os::SleepThread(ams::TimeSpan::FromMilliSeconds(20));
        }

        this->MakeSwitch2Command(&report, 0x16, 0x01, nullptr, 0);
        R_TRY(this->WriteDataReport(&report));
        os::SleepThread(ams::TimeSpan::FromMilliSeconds(20));

        const u8 vibration_sample[] = { 0x03, 0x00, 0x00, 0x00 };
        this->MakeSwitch2Command(&report, 0x0A, 0x02, vibration_sample, sizeof(vibration_sample));
        R_TRY(this->WriteDataReport(&report));
        os::SleepThread(ams::TimeSpan::FromMilliSeconds(50));

        // Set Player LEDs. The captured sequence sends an 8-byte LED payload.
        const u8 led_data[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        this->MakeSwitch2Command(&report, 0x09, 0x07, led_data, sizeof(led_data));
        R_TRY(this->WriteDataReport(&report));
        os::SleepThread(ams::TimeSpan::FromMilliSeconds(50));

        const u8 feature_mask[] = { this->GetFeatureMask(), 0x00, 0x00, 0x00 };

        this->MakeSwitch2Command(&report, 0x0C, 0x02, feature_mask, sizeof(feature_mask));
        R_TRY(this->WriteDataReport(&report));
        os::SleepThread(ams::TimeSpan::FromMilliSeconds(50));

        this->MakeSwitch2Command(&report, 0x0C, 0x04, feature_mask, sizeof(feature_mask));
        R_TRY(this->WriteDataReport(&report));

        R_SUCCEED();
    }

    void Switch2Controller::ProcessInputData(const bluetooth::HidReport *report) {
        // Joy2Win and newer firmware paths primarily stream universal report 0x05.
        // Prefer decoding that format whenever we have enough bytes, then fall back
        // to type-specific default reports for shorter legacy payloads.
        if (report->size >= Switch2InputReport0x05MinLength) {
            this->MapInputReport0x05(report);
            return;
        }

        switch (m_id.pid) {
            case 0x2060:
                this->MapInputReport0x07(report);
                break;
            case 0x2061:
                this->MapInputReport0x08(report);
                break;
            case 0x2062:
                this->MapInputReport0x09(report);
                break;
            case 0x2064:
                this->MapInputReport0x0A(report);
                break;
            default:
                this->MapInputReport0x05(report);
                break;
        }
    }

    void Switch2Controller::MapInputReport0x05(const bluetooth::HidReport *report) {
        // Every Switch 2 controller delivers the same universal report 0x05 over
        // its BLE input characteristic. Guard against short/garbage packets.
        if (report->size < Switch2InputReport0x05MinLength) {
            return;
        }

        this->ResetSwitch2StateForReport();

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

        if (report->size > Switch2InputReport0x05Offset_BatteryVoltage + 1) {
            const u16 voltage_mv = static_cast<u16>(report->data[Switch2InputReport0x05Offset_BatteryVoltage] |
                                                   (report->data[Switch2InputReport0x05Offset_BatteryVoltage + 1] << 8));
            m_battery = convert_battery_voltage(voltage_mv);
        }
    }

    void Switch2Controller::MapInputReport0x07(const bluetooth::HidReport *report) {
        if (report->size < Switch2InputReport0x07MinLength) {
            return;
        }

        this->ResetSwitch2StateForReport();
        this->ApplyPowerInfo(report->data[0x01]);

        const u8 b0 = report->data[0x02];
        const u8 b1 = report->data[0x03];

        m_buttons.dpad_down    = (b0 & 0x01) != 0;
        m_buttons.dpad_right   = (b0 & 0x02) != 0;
        m_buttons.dpad_left    = (b0 & 0x04) != 0;
        m_buttons.dpad_up      = (b0 & 0x08) != 0;
        m_buttons.L            = (b0 & 0x10) != 0;
        m_buttons.ZL           = (b0 & 0x20) != 0;
        m_buttons.minus        = (b0 & 0x40) != 0;
        m_buttons.lstick_press = (b0 & 0x80) != 0;
        m_buttons.capture      = (b1 & 0x01) != 0;

        m_left_stick.SetData(
            Unpack12BitStickX(&report->data[0x05]),
            Unpack12BitStickY(&report->data[0x05])
        );
    }

    void Switch2Controller::MapInputReport0x08(const bluetooth::HidReport *report) {
        if (report->size < Switch2InputReport0x08MinLength) {
            return;
        }

        this->ResetSwitch2StateForReport();
        this->ApplyPowerInfo(report->data[0x01]);

        const u8 b0 = report->data[0x02];
        const u8 b1 = report->data[0x03];

        m_buttons.B            = (b0 & 0x01) != 0;
        m_buttons.A            = (b0 & 0x02) != 0;
        m_buttons.Y            = (b0 & 0x04) != 0;
        m_buttons.X            = (b0 & 0x08) != 0;
        m_buttons.R            = (b0 & 0x10) != 0;
        m_buttons.ZR           = (b0 & 0x20) != 0;
        m_buttons.plus         = (b0 & 0x40) != 0;
        m_buttons.rstick_press = (b0 & 0x80) != 0;
        m_buttons.home         = (b1 & 0x01) != 0;

        m_right_stick.SetData(
            Unpack12BitStickX(&report->data[0x05]),
            Unpack12BitStickY(&report->data[0x05])
        );
    }

    void Switch2Controller::MapInputReport0x09(const bluetooth::HidReport *report) {
        if (report->size < Switch2InputReport0x09MinLength) {
            return;
        }

        this->ResetSwitch2StateForReport();
        this->ApplyPowerInfo(report->data[0x01]);

        const u8 b0 = report->data[0x02];
        const u8 b1 = report->data[0x03];
        const u8 b2 = report->data[0x04];

        m_buttons.B            = (b0 & 0x01) != 0;
        m_buttons.A            = (b0 & 0x02) != 0;
        m_buttons.Y            = (b0 & 0x04) != 0;
        m_buttons.X            = (b0 & 0x08) != 0;
        m_buttons.R            = (b0 & 0x10) != 0;
        m_buttons.ZR           = (b0 & 0x20) != 0;
        m_buttons.plus         = (b0 & 0x40) != 0;
        m_buttons.rstick_press = (b0 & 0x80) != 0;

        m_buttons.dpad_down    = (b1 & 0x01) != 0;
        m_buttons.dpad_right   = (b1 & 0x02) != 0;
        m_buttons.dpad_left    = (b1 & 0x04) != 0;
        m_buttons.dpad_up      = (b1 & 0x08) != 0;
        m_buttons.L            = (b1 & 0x10) != 0;
        m_buttons.ZL           = (b1 & 0x20) != 0;
        m_buttons.minus        = (b1 & 0x40) != 0;
        m_buttons.lstick_press = (b1 & 0x80) != 0;

        m_buttons.home         = (b2 & 0x01) != 0;
        m_buttons.capture      = (b2 & 0x02) != 0;

        m_left_stick.SetData(
            Unpack12BitStickX(&report->data[0x05]),
            Unpack12BitStickY(&report->data[0x05])
        );
        m_right_stick.SetData(
            Unpack12BitStickX(&report->data[0x08]),
            Unpack12BitStickY(&report->data[0x08])
        );
    }

    void Switch2Controller::MapInputReport0x0A(const bluetooth::HidReport *report) {
        if (report->size < Switch2InputReport0x0AMinLength) {
            return;
        }

        this->ResetSwitch2StateForReport();
        this->ApplyPowerInfo(report->data[0x01]);

        const u8 b0 = report->data[0x02];
        const u8 b1 = report->data[0x03];
        const u8 b2 = report->data[0x04];

        m_buttons.B            = (b0 & 0x01) != 0;
        m_buttons.A            = (b0 & 0x02) != 0;
        m_buttons.Y            = (b0 & 0x04) != 0;
        m_buttons.X            = (b0 & 0x08) != 0;
        m_buttons.ZR           = (b0 & 0x10) != 0;
        m_buttons.R            = (b0 & 0x20) != 0;
        m_buttons.plus         = (b0 & 0x40) != 0;
        m_buttons.rstick_press = (b0 & 0x80) != 0;

        m_buttons.dpad_down    = (b1 & 0x01) != 0;
        m_buttons.dpad_right   = (b1 & 0x02) != 0;
        m_buttons.dpad_left    = (b1 & 0x04) != 0;
        m_buttons.dpad_up      = (b1 & 0x08) != 0;
        m_buttons.ZL           = (b1 & 0x10) != 0;
        m_buttons.L            = (b1 & 0x20) != 0;
        m_buttons.minus        = (b1 & 0x40) != 0;
        m_buttons.lstick_press = (b1 & 0x80) != 0;

        m_buttons.home         = (b2 & 0x01) != 0;
        m_buttons.capture      = (b2 & 0x02) != 0;

        m_left_stick.SetData(
            Unpack12BitStickX(&report->data[0x05]),
            Unpack12BitStickY(&report->data[0x05])
        );
        m_right_stick.SetData(
            Unpack12BitStickX(&report->data[0x08]),
            Unpack12BitStickY(&report->data[0x08])
        );

        m_buttons.ZL |= report->data[0x0C] > Switch2TriggerThreshold ? 1 : 0;
        m_buttons.ZR |= report->data[0x0D] > Switch2TriggerThreshold ? 1 : 0;
    }

    // --- NSO GameCube Controller 2 ---

    void NSOGCController2Controller::ProcessInputData(const bluetooth::HidReport *report) {
        this->MapInputReport0x0A(report);
    }

}

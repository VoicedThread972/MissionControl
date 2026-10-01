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
#include "switch2_hdls.hpp"
#include "switch2_protocol.hpp"
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

        // Approximate usable deflection of the 12-bit sticks around 0x800. Values
        // beyond it are clamped; factory calibration is not read yet.
        constexpr s32 StickDeflection = 1600;

        s32 convert_stick_axis(u16 raw) {
            const s32 value = (static_cast<s32>(raw) - static_cast<s32>(SwitchAnalogStick::Center)) * JOYSTICK_MAX / StickDeflection;
            return std::clamp<s32>(value, -JOYSTICK_MAX, JOYSTICK_MAX);
        }

        switch2_hdls::PadKind pad_kind_from_pid(u16 pid) {
            switch (pid) {
                case switch2::PidJoyConL: return switch2_hdls::PadKind::JoyConLeft;
                case switch2::PidJoyConR: return switch2_hdls::PadKind::JoyConRight;
                default:                  return switch2_hdls::PadKind::FullKey;
            }
        }

    }

    // --- Switch2Controller base ---

    Switch2Controller::~Switch2Controller() {
        switch2_hdls::Detach(this);
    }

    Result Switch2Controller::HandleDataReportEvent(const bluetooth::HidReportEventInfo *event_info) {
        // The BLE bridge always fills the v9 layout.
        HiddbgHdlsState state;
        {
            std::scoped_lock lk(m_input_mutex);
            this->ProcessInputData(&event_info->data_report.v9.report);
            this->BuildHdlsState(&state);
        }
        switch2_hdls::Update(this, state);
        R_SUCCEED();
    }

    void Switch2Controller::BuildHdlsState(HiddbgHdlsState *out) {
        *out = {};
        out->battery_level = std::min<u32>(m_battery / 2, 4);
        out->flags = (m_ext_power ? BIT(0) : 0) | (m_charging ? BIT(1) : 0);

        u64 b = 0;
        if (m_buttons.A)            b |= HidNpadButton_A;
        if (m_buttons.B)            b |= HidNpadButton_B;
        if (m_buttons.X)            b |= HidNpadButton_X;
        if (m_buttons.Y)            b |= HidNpadButton_Y;
        if (m_buttons.lstick_press) b |= HidNpadButton_StickL;
        if (m_buttons.rstick_press) b |= HidNpadButton_StickR;
        if (m_buttons.L)            b |= HidNpadButton_L;
        if (m_buttons.R)            b |= HidNpadButton_R;
        if (m_buttons.ZL)           b |= HidNpadButton_ZL;
        if (m_buttons.ZR)           b |= HidNpadButton_ZR;
        if (m_buttons.plus)         b |= HidNpadButton_Plus;
        if (m_buttons.minus)        b |= HidNpadButton_Minus;
        if (m_buttons.dpad_left)    b |= HidNpadButton_Left;
        if (m_buttons.dpad_up)      b |= HidNpadButton_Up;
        if (m_buttons.dpad_right)   b |= HidNpadButton_Right;
        if (m_buttons.dpad_down)    b |= HidNpadButton_Down;
        // hid masks bits 20-27 of HDLS input (libnx hiddbg.h), so SL/SR may be dropped.
        if (m_buttons.SL_left)      b |= HidNpadButton_LeftSL;
        if (m_buttons.SR_left)      b |= HidNpadButton_LeftSR;
        if (m_buttons.SL_right)     b |= HidNpadButton_RightSL;
        if (m_buttons.SR_right)     b |= HidNpadButton_RightSR;
        if (m_buttons.home)         b |= HiddbgNpadButton_Home;
        if (m_buttons.capture)      b |= HiddbgNpadButton_Capture;
        out->buttons = b;

        out->analog_stick_l.x = convert_stick_axis(m_left_stick.GetX());
        out->analog_stick_l.y = convert_stick_axis(m_left_stick.GetY());
        out->analog_stick_r.x = convert_stick_axis(m_right_stick.GetX());
        out->analog_stick_r.y = convert_stick_axis(m_right_stick.GetY());
    }

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
            case switch2::PidJoyConL:
            case switch2::PidJoyConR:
                return FeatureMaskJoyCon2;
            case switch2::PidPro:
                return FeatureMaskPro2;
            case switch2::PidNsoGc:
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

        bluetooth::HidReport report;
        u32 pairing_step = 1;

        SW2_LOG_INFO("[S4][PAIR-00] Start command bootstrap (no persistent bonding) vid=0x%04X pid=0x%04X", m_id.vid, m_id.pid);

        auto send_pairing_step = [&](const char *instruction, u8 cmd_id, u8 sub_id, const u8 *data, u8 data_len, u32 delay_ms) -> Result {
            const u32 current_step = pairing_step++;

            SW2_LOG_INFO("[S4][PAIR-%02u] %s", current_step, instruction);
            this->MakeSwitch2Command(&report, cmd_id, sub_id, data, data_len);

            const Result rc = this->WriteDataReport(&report);
            if (R_FAILED(rc)) {
                SW2_LOG_WARN("[S4][FAIL][PAIR-%02u] cmd=0x%02X sub=0x%02X " SW2_RC_FMT " (bootstrap aborted; see preceding [S4][FAIL][CMD-*])",
                    current_step, cmd_id, sub_id, SW2_RC_ARGS(rc));
                return rc;
            }

            // WriteDataReport returns only after a successful ack for commands that expect one.
            SW2_LOG_INFO("[S4][OK][PAIR-%02u] cmd=0x%02X sub=0x%02X done", current_step, cmd_id, sub_id);

            if (delay_ms != 0) {
                os::SleepThread(ams::TimeSpan::FromMilliSeconds(delay_ms));
            }

            R_SUCCEED();
        };

        // Observed early bootstrap commands from the captured Bluetooth sequence.
        R_TRY(send_pairing_step(
            "Send observed bootstrap command 07/01 (purpose unknown).",
            0x07,
            0x01,
            nullptr,
            0,
            20
        ));

        // Pairing is optional for GATT input (documentation/bluetooth_interface.md).
        // Do not commit trace keys or overwrite the controller's saved hosts.
        // Persistent bonding requires verified B1/B2, host-side key storage and
        // a supported link-encryption path; none may be replaced by replay data.

        if (m_id.pid == switch2::PidJoyConL || m_id.pid == switch2::PidJoyConR || m_id.pid == switch2::PidNsoGc) {
            R_TRY(send_pairing_step(
                "Request firmware information.",
                0x10,
                0x01,
                nullptr,
                0,
                20
            ));
        }

        R_TRY(send_pairing_step(
            "Send observed bootstrap command 16/01 (purpose unknown).",
            0x16,
            0x01,
            nullptr,
            0,
            20
        ));

        const u8 vibration_sample[] = { 0x03, 0x00, 0x00, 0x00 };
        R_TRY(send_pairing_step(
            "Play vibration sample 3 (not calibration).",
            0x0A,
            0x02,
            vibration_sample,
            sizeof(vibration_sample),
            50
        ));

        // Set Player LEDs. The captured sequence sends an 8-byte LED payload.
        const u8 led_data[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        R_TRY(send_pairing_step(
            "Set player LED 1 (not proof of Horizon registration).",
            0x09,
            0x07,
            led_data,
            sizeof(led_data),
            50
        ));

        const u8 feature_mask[] = { this->GetFeatureMask(), 0x00, 0x00, 0x00 };

        R_TRY(send_pairing_step(
            "Apply feature mask phase 1.",
            0x0C,
            0x02,
            feature_mask,
            sizeof(feature_mask),
            50
        ));

        R_TRY(send_pairing_step(
            "Apply feature mask phase 2; no pairing keys are stored.",
            0x0C,
            0x04,
            feature_mask,
            sizeof(feature_mask),
            0
        ));

        SW2_LOG_INFO("[S4][OK][GATT-INIT] command bootstrap acknowledged; registering with Horizon via HDLS ([S6])");

        switch2_hdls::Attach(this, m_address, pad_kind_from_pid(m_id.pid));

        R_SUCCEED();
    }

    void Switch2Controller::ProcessInputData(const bluetooth::HidReport *report) {
        if (report->size < 1 || report->size > sizeof(report->data) ||
            !switch2::ValidInput(report->data[0], m_id.pid, report->size - 1)) {
            if (!m_logged_invalid_input) {
                m_logged_invalid_input = true;
                SW2_LOG_WARN("[S5][DROP][INVALID-INPUT] report=0x%02X pid=0x%04X size=%u rejected by decoder (first only)",
                    report->size >= 1 ? report->data[0] : 0, m_id.pid, report->size);
            }
            return;
        }

        // The BLE bridge restores the report ID; decoders below use payload
        // offsets from the documentation. This scratch buffer is used only
        // under SwitchController::m_input_mutex, like the rest of input state.
        m_payload_report.size = report->size - 1;
        std::memcpy(m_payload_report.data, report->data + 1, m_payload_report.size);
        switch (report->data[0]) {
            case 0x05: this->MapInputReport0x05(&m_payload_report); break;
            case 0x07: this->MapInputReport0x07(&m_payload_report); break;
            case 0x08: this->MapInputReport0x08(&m_payload_report); break;
            case 0x09: this->MapInputReport0x09(&m_payload_report); break;
            case 0x0A: this->MapInputReport0x0A(&m_payload_report); break;
        }

        // Decoded state changes prove the mapping works on hardware without logging every packet.
        u32 buttons = 0;
        std::memcpy(&buttons, &m_buttons, sizeof(m_buttons));
        if (buttons != m_last_logged_buttons) {
            m_last_logged_buttons = buttons;
            SW2_LOG_VERBOSE("[S5][INPUT-CHANGE] report=0x%02X buttons=%06X ls=%u,%u rs=%u,%u battery=%u",
                report->data[0], buttons,
                m_left_stick.GetX(), m_left_stick.GetY(), m_right_stick.GetX(), m_right_stick.GetY(), m_battery);
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
        m_buttons.SL_left      = btn.SL_left;
        m_buttons.SR_left      = btn.SR_left;
        m_buttons.SL_right     = btn.SL_right;
        m_buttons.SR_right     = btn.SR_right;

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
        if (m_id.pid == switch2::PidNsoGc && report->size > Switch2InputReport0x05Offset_TriggerR) {
            m_buttons.ZL |= report->data[Switch2InputReport0x05Offset_TriggerL] > Switch2TriggerThreshold;
            m_buttons.ZR |= report->data[Switch2InputReport0x05Offset_TriggerR] > Switch2TriggerThreshold;
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
        m_buttons.SR_left      = (b1 & 0x40) != 0;
        m_buttons.SL_left      = (b1 & 0x80) != 0;

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
        m_buttons.SR_right     = (b1 & 0x40) != 0;
        m_buttons.SL_right     = (b1 & 0x80) != 0;

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
        this->Switch2Controller::ProcessInputData(report);
    }

}

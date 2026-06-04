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
#include "controller_management.hpp"
#include "switch2_debug.hpp"
#include "switch2_discovery.hpp"
#include <stratosphere.hpp>
#include "../utils.hpp"

namespace ams::controller {

    namespace {

        const std::string OfficialGamepadNames[] = {
            "NintendoGamepad",
            "Joy-Con",
            "Pro Controller",
            "Lic Pro Controller",
            "NES Controller",
            "HVC Controller",
            "SNES Controller",
            "N64 Controller",
            "MD/Gen Control Pad",
            "Lic2 Pro Controller",
            "Lic3 Pro Controller",
        };

        constexpr u8 DeviceClassMajorPeripheral = 0x05;
        constexpr u8 DeviceClassMinorGamepad    = 0x08;
        constexpr u8 DeviceClassMinorJoystick   = 0x04;
        constexpr u8 DeviceClassMinorKeyboard   = 0x40;

        constinit os::SdkMutex g_controller_lock;
        std::vector<std::shared_ptr<SwitchController>> g_controllers;

        void FormatAddress(char *buf, size_t bufsz, bluetooth::Address address) {
            util::SNPrintf(
                buf,
                bufsz,
                "%02X:%02X:%02X:%02X:%02X:%02X",
                address.address[0], address.address[1], address.address[2],
                address.address[3], address.address[4], address.address[5]
            );
        }

        const char *GetControllerTypeName(ControllerType type) {
            switch (type) {
                case ControllerType_Switch2JoyConL:        return "Switch2JoyConL";
                case ControllerType_Switch2JoyConR:        return "Switch2JoyConR";
                case ControllerType_Switch2ProController:  return "Switch2ProController";
                case ControllerType_Switch2NSOGCController:return "Switch2NSOGCController";
                case ControllerType_Switch:                return "Switch";
                case ControllerType_Unknown:               return "Unknown";
                default:                                   return "Other";
            }
        }

        bool CreateSwitch2ControllerFromType(ControllerType type, bluetooth::Address address, HardwareID *id, std::shared_ptr<SwitchController> *out_controller) {
            AMS_ABORT_UNLESS(id != nullptr);
            AMS_ABORT_UNLESS(out_controller != nullptr);

            switch (type) {
                case ControllerType_Switch2JoyConL:
                    if (id->vid == 0 || id->pid == 0) {
                        *id = JoyCon2LController::hardware_ids[0];
                    }
                    *out_controller = std::make_shared<JoyCon2LController>(address, *id);
                    return true;
                case ControllerType_Switch2JoyConR:
                    if (id->vid == 0 || id->pid == 0) {
                        *id = JoyCon2RController::hardware_ids[0];
                    }
                    *out_controller = std::make_shared<JoyCon2RController>(address, *id);
                    return true;
                case ControllerType_Switch2ProController:
                    if (id->vid == 0 || id->pid == 0) {
                        *id = ProController2Controller::hardware_ids[0];
                    }
                    *out_controller = std::make_shared<ProController2Controller>(address, *id);
                    return true;
                case ControllerType_Switch2NSOGCController:
                    if (id->vid == 0 || id->pid == 0) {
                        *id = NSOGCController2Controller::hardware_ids[0];
                    }
                    *out_controller = std::make_shared<NSOGCController2Controller>(address, *id);
                    return true;
                default:
                    return false;
            }
        }

    }

    ControllerType Identify(const bluetooth::DevicesSettings *device) {
        // Check Switch 2 controllers FIRST to avoid misidentification by name
        for (auto hwId : JoyCon2LController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Switch2JoyConL;
            }
        }

        for (auto hwId : JoyCon2RController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Switch2JoyConR;
            }
        }

        for (auto hwId : ProController2Controller::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Switch2ProController;
            }
        }

        for (auto hwId : NSOGCController2Controller::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Switch2NSOGCController;
            }
        }

        for (auto hwId : SwitchController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Switch;
            }
        }

        const char *controller_name = hos::GetVersion() < hos::Version_13_0_0 ? device->name.name : device->name2;

        // Additionally check controller name against known official Nintendo controllers, as some controllers (eg. JoyCons paired via rails) don't report the correct vid/pid
        if (IsOfficialSwitchControllerName(controller_name))
            return ControllerType_Switch;

        for (auto hwId : WiiController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Wii;
            }
        }

        for (auto hwId : Dualshock3Controller::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Dualshock3;
            }
        }

        for (auto hwId : Dualshock4Controller::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Dualshock4;
            }
        }

        for (auto hwId : DualsenseController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Dualsense;
            }
        }

        for (auto hwId : XboxOneController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_XboxOne;
            }
        }

        for (auto hwId : OuyaController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Ouya;
            }
        }

        for (auto hwId : GamestickController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Gamestick;
            }
        }

        for (auto hwId : GemboxController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Gembox;
            }
        }

        for (auto hwId : IpegaController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                if (std::strcmp(controller_name, AmazonController::FireGameControllerName) ==  0) {
                    return ControllerType_Amazon;
                } else {
                    return ControllerType_Ipega;
                }
            }
        }

        for (auto hwId : XiaomiController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Xiaomi;
            }
        }

        for (auto hwId : GamesirController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Gamesir;
            }
        }

        for (auto hwId : SteelseriesController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Steelseries;
            }
        }

        for (auto hwId : NvidiaShieldController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_NvidiaShield;
          }
        }

        for (auto hwId : EightBitDoController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_8BitDo;
            }
        }

        for (auto hwId : PowerAController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_PowerA;
            }
        }

        for (auto hwId : MadCatzController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_MadCatz;
            }
        }

        for (auto hwId : MocuteController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Mocute;
            }
        }

        for (auto hwId : RazerController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Razer;
            }
        }

        for (auto hwId : ICadeController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_ICade;
            }
        }

        for (auto hwId : LanShenController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_LanShen;
            }
        }

        for (auto hwId : AtGamesController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_AtGames;
            }
        }

        for (auto hwId : HyperkinController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Hyperkin;
            }
        }

        for (auto hwId : BetopController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Betop;
            }
        }

        for (auto hwId : AtariController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Atari;
            }
        }

        for (auto hwId : BionikController::hardware_ids) {
            if ( (device->vid == hwId.vid) && (device->pid == hwId.pid) ) {
                return ControllerType_Bionik;
            }
        }

        return ControllerType_Unknown;
    }

    bool IsAllowedDeviceClass(const bluetooth::DeviceClass *cod) {
        return ((cod->class_of_device[1] & 0x0f) == DeviceClassMajorPeripheral) &&
               (((cod->class_of_device[2] & 0x0f) == DeviceClassMinorGamepad) || ((cod->class_of_device[2] & 0x0f) == DeviceClassMinorJoystick) || ((cod->class_of_device[2] & 0x40) == DeviceClassMinorKeyboard));
    }

    bool IsOfficialSwitchControllerName(const std::string& name) {
        for (auto n : OfficialGamepadNames) {
            if (name.rfind(n, 0) == 0)
                return true;
        }

        return false;
    }

    void AttachHandler(bluetooth::Address address) {
        bluetooth::DevicesSettings device_settings;
        Result r = btdrvGetPairedDeviceInfo(address, &device_settings);

        char addr_str[20];
        FormatAddress(addr_str, sizeof(addr_str), address);
        SW2_LOG_INFO("AttachHandler: addr=%s", addr_str);

        HardwareID id = { 0, 0 };

        std::shared_ptr<SwitchController> controller;
        bool is_switch2_handler = false;

        auto create_switch2_controller = [&](ControllerType type) {
            if (CreateSwitch2ControllerFromType(type, address, &id, &controller)) {
                is_switch2_handler = true;
                return true;
            }

            return false;
        };

        if (R_SUCCEEDED(r)) {
            id = { device_settings.vid, device_settings.pid };
            const ControllerType detected_type = Identify(&device_settings);

            SW2_LOG_INFO(
                "AttachHandler paired-info: addr=%s vid=0x%04X pid=0x%04X detected=%s(%u)",
                addr_str,
                id.vid,
                id.pid,
                GetControllerTypeName(detected_type),
                static_cast<u32>(detected_type)
            );

            switch (detected_type) {
            case ControllerType_Switch:
                controller = std::make_shared<SwitchController>(address, id);
                break;
            case ControllerType_Wii:
                controller = std::make_shared<WiiController>(address, id);
                break;
            case ControllerType_Dualshock3:
                controller = std::make_shared<Dualshock3Controller>(address, id);
                break;
            case ControllerType_Dualshock4:
                controller = std::make_shared<Dualshock4Controller>(address, id);
                break;
            case ControllerType_Dualsense:
                controller = std::make_shared<DualsenseController>(address, id);
                break;
            case ControllerType_XboxOne:
                controller = std::make_shared<XboxOneController>(address, id);
                break;
            case ControllerType_Ouya:
                controller = std::make_shared<OuyaController>(address, id);
                break;
            case ControllerType_Gamestick:
                controller = std::make_shared<GamestickController>(address, id);
                break;
            case ControllerType_Gembox:
                controller = std::make_shared<GemboxController>(address, id);
                break;
            case ControllerType_Ipega:
                controller = std::make_shared<IpegaController>(address, id);
                break;
            case ControllerType_Xiaomi:
                controller = std::make_shared<XiaomiController>(address, id);
                break;
            case ControllerType_Gamesir:
                controller = std::make_shared<GamesirController>(address, id);
                break;
            case ControllerType_Steelseries:
                controller = std::make_shared<SteelseriesController>(address, id);
                break;
            case ControllerType_NvidiaShield:
                controller = std::make_shared<NvidiaShieldController>(address, id);
                break;
            case ControllerType_8BitDo:
                controller = std::make_shared<EightBitDoController>(address, id);
                break;
            case ControllerType_PowerA:
                controller = std::make_shared<PowerAController>(address, id);
                break;
            case ControllerType_MadCatz:
                controller = std::make_shared<MadCatzController>(address, id);
                break;
            case ControllerType_Mocute:
                controller = std::make_shared<MocuteController>(address, id);
                break;
            case ControllerType_Razer:
                controller = std::make_shared<RazerController>(address, id);
                break;
            case ControllerType_ICade:
                controller = std::make_shared<ICadeController>(address, id);
                break;
            case ControllerType_LanShen:
                controller = std::make_shared<LanShenController>(address, id);
                break;
            case ControllerType_AtGames:
                controller = std::make_shared<AtGamesController>(address, id);
                break;
            case ControllerType_Hyperkin:
                controller = std::make_shared<HyperkinController>(address, id);
                break;
            case ControllerType_Betop:
                controller = std::make_shared<BetopController>(address, id);
                break;
            case ControllerType_Atari:
                controller = std::make_shared<AtariController>(address, id);
                break;
            case ControllerType_Bionik:
                controller = std::make_shared<BionikController>(address, id);
                break;
            case ControllerType_Amazon:
                controller = std::make_shared<AmazonController>(address, id);
                break;
            case ControllerType_Switch2JoyConL:
                create_switch2_controller(ControllerType_Switch2JoyConL);
                break;
            case ControllerType_Switch2JoyConR:
                create_switch2_controller(ControllerType_Switch2JoyConR);
                break;
            case ControllerType_Switch2ProController:
                create_switch2_controller(ControllerType_Switch2ProController);
                break;
            case ControllerType_Switch2NSOGCController:
                create_switch2_controller(ControllerType_Switch2NSOGCController);
                break;
            default:
                // Some BLE devices have incomplete paired-info metadata even when
                // we know their type from advertisement discovery.
                if (!create_switch2_controller(GetDiscoveredSwitch2ControllerType(address))) {
                    controller = std::make_shared<UnknownController>(address, id);
                }
                break;
            }
        } else {
            // Paired device info is not available. This is expected for Switch 2
            // controllers, which connect over BLE and may not have classic paired
            // device settings. If the BLE discovery layer has identified the address
            // as a known Switch 2 controller, attach the matching handler. Otherwise
            // fall back to a passthrough UnknownController (matching the master
            // default for unidentified devices) rather than guessing a Switch handler.
            SW2_LOG_WARN("Paired device info unavailable for addr=%s (rc=0x%08X); checking Switch 2 discovery map", addr_str, static_cast<u32>(r.GetValue()));
            const ControllerType discovered_type = GetDiscoveredSwitch2ControllerType(address);
            SW2_LOG_INFO("AttachHandler discovery-type: addr=%s type=%s(%u)", addr_str, GetControllerTypeName(discovered_type), static_cast<u32>(discovered_type));

            if (!create_switch2_controller(discovered_type)) {
                controller = std::make_shared<UnknownController>(address, id);
            }
        }

        SW2_LOG_INFO(
            "AttachHandler selected: addr=%s switch2=%u vid=0x%04X pid=0x%04X",
            addr_str,
            static_cast<u32>(is_switch2_handler),
            id.vid,
            id.pid
        );

        {
            std::scoped_lock lk(g_controller_lock);
            g_controllers.push_back(controller);
        }

        if (R_FAILED(controller->Initialize())) {
            if (is_switch2_handler) {
                // Switch 2 currently receives input through BLE GATT notify routing,
                // not classic HID report channels. Keep the handler alive so BLE
                // notifications can still be decoded.
                SW2_LOG_WARN("Switch2 handler init failed on classic HID path; keeping BLE handler active");
            } else {
                SW2_LOG_WARN("Controller init failed for addr=%s; closing HID connection", addr_str);
                // Try to disconnect the controller
                btdrvCloseHidConnection(controller->Address());
            }
        }
    }

    void RemoveHandler(bluetooth::Address address) {
        std::scoped_lock lk(g_controller_lock);

        for (auto it = g_controllers.begin(); it < g_controllers.end(); ++it) {
            if (utils::BluetoothAddressCompare((*it)->Address(), address)) {
                g_controllers.erase(it);
                return;
            }
        }
    }

    std::shared_ptr<SwitchController> LocateHandler(bluetooth::Address address) {
        std::scoped_lock lk(g_controller_lock);

        for (auto it = g_controllers.begin(); it < g_controllers.end(); ++it) {
            if (utils::BluetoothAddressCompare((*it)->Address(), address)) {
                return (*it);
            }
        }

        return nullptr;
    }

}

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
#include <switch.h>
#include <stratosphere.hpp>
#include "../bluetooth_mitm/bluetooth/bluetooth_types.hpp"

namespace ams::controller::switch2_hdls {

    /*
     * Registers Switch 2 controllers with Horizon as hid:dbg HDLS virtual
     * devices (the mechanism sys-con uses). Switch 2 controllers talk BLE GATT,
     * which Horizon's classic-HID pairing path cannot represent, so a virtual
     * npad is what makes them visible in "Change Grip/Order".
     *
     * All hid:dbg IPC runs on a dedicated worker thread. These functions only
     * update shared state and signal it, so they are safe to call from the BLE
     * event thread. Slots are keyed by an owner token (the controller object).
     */
    enum class PadKind : u8 {
        FullKey,
        JoyConLeft,
        JoyConRight,
    };

    void Attach(const void *owner, const bluetooth::Address &address, PadKind kind);
    void Update(const void *owner, const HiddbgHdlsState &state);
    void Detach(const void *owner);

}

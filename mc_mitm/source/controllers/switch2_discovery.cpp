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
#include "switch2_discovery.hpp"
#include "../utils.hpp"
#include <map>
#include <stratosphere.hpp>

namespace ams::controller {

    namespace {
        
        struct AddressCompare {
            bool operator()(const bluetooth::Address& lhs, const bluetooth::Address& rhs) const {
                for (int i = 0; i < 6; ++i) {
                    if (lhs.address[i] < rhs.address[i]) return true;
                    if (lhs.address[i] > rhs.address[i]) return false;
                }
                return false;
            }
        };

        constinit os::SdkMutex g_discovery_lock;
        std::map<bluetooth::Address, ControllerType, AddressCompare> g_discovered_controllers;

    }

    void RegisterDiscoveredSwitch2Controller(const bluetooth::Address &address, ControllerType type) {
        std::scoped_lock lk(g_discovery_lock);
        g_discovered_controllers[address] = type;
    }

    ControllerType GetDiscoveredSwitch2ControllerType(const bluetooth::Address &address) {
        std::scoped_lock lk(g_discovery_lock);
        auto it = g_discovered_controllers.find(address);
        if (it != g_discovered_controllers.end()) {
            return it->second;
        }
        return ControllerType_Unknown;
    }

}

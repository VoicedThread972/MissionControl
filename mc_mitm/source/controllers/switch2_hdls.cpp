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
#include "switch2_hdls.hpp"
#include "switch2_debug.hpp"
#include <cstring>

namespace ams::controller::switch2_hdls {

    namespace {

        constexpr size_t MaxPads = 4;
        constexpr u32 MaxAttachAttempts = 3;
        constexpr u32 MaxSetFailureLogs = 4;

        struct Pad {
            const void *owner;
            bluetooth::Address address;
            PadKind kind;
            bool used;        // Owned by a live controller.
            bool busy;        // Worker IPC in flight; slot must not be reused.
            bool attached;    // Written by the worker only.
            bool dirty;
            u32 attach_attempts;
            u32 set_failures;
            HiddbgHdlsHandle handle;
            HiddbgHdlsState state;
        };

        constinit os::SdkMutex g_lock;
        constinit Pad g_pads[MaxPads] = {};
        os::Event g_event(os::EventClearMode_AutoClear);

        // hid:dbg maps this buffer as transfer memory; it must be page aligned.
        alignas(0x1000) constinit u8 g_work_buffer[0x1000] = {};
        constinit HiddbgHdlsSessionId g_session = {};
        constinit bool g_hdls_ready = false;      // Worker thread only.
        constinit u32 g_init_failures = 0;        // Worker thread only.

        constexpr s32 WorkerThreadPriority = 20;
        constexpr size_t WorkerThreadStackSize = 0x2000;
        alignas(os::ThreadStackAlignment) constinit u8 g_worker_thread_stack[WorkerThreadStackSize];
        constinit os::ThreadType g_worker_thread;
        constinit bool g_worker_started = false;  // Guarded by g_lock.

        constexpr u32 Rgb(u8 r, u8 g, u8 b) {
            return static_cast<u32>(r) | (static_cast<u32>(g) << 8) | (static_cast<u32>(b) << 16) | (0xFFu << 24);
        }

        const char *PadKindName(PadKind kind) {
            switch (kind) {
                case PadKind::JoyConLeft:  return "JoyLeft2";
                case PadKind::JoyConRight: return "JoyRight1";
                default:                   return "FullKey3";
            }
        }

        HiddbgHdlsDeviceInfo MakeDeviceInfo(PadKind kind) {
            HiddbgHdlsDeviceInfo info = {};
            switch (kind) {
                case PadKind::JoyConLeft:
                    info.deviceType = HidDeviceType_JoyLeft2;
                    info.singleColorBody = Rgb(0x00, 0xA8, 0xE0);
                    info.singleColorButtons = Rgb(0x1E, 0x1E, 0x1E);
                    break;
                case PadKind::JoyConRight:
                    info.deviceType = HidDeviceType_JoyRight1;
                    info.singleColorBody = Rgb(0xF0, 0x45, 0x3A);
                    info.singleColorButtons = Rgb(0x1E, 0x1E, 0x1E);
                    break;
                default:
                    info.deviceType = HidDeviceType_FullKey3;
                    info.singleColorBody = Rgb(0x32, 0x32, 0x32);
                    info.singleColorButtons = Rgb(0xE6, 0xE6, 0xE6);
                    break;
            }
            info.colorLeftGrip = info.singleColorBody;
            info.colorRightGrip = info.singleColorBody;
            // Bluetooth = dedicated controller (Rail would merge into a handheld pad).
            info.npadInterfaceType = HidNpadInterfaceType_Bluetooth;
            return info;
        }

        bool EnsureHdlsReady() {
            if (g_hdls_ready) return true;
            if (g_init_failures >= MaxAttachAttempts) return false;

            Result rc = hiddbgInitialize();
            if (R_FAILED(rc)) {
                ++g_init_failures;
                SW2_LOG_WARN("[S6][FAIL][HDLS-INIT] hiddbgInitialize " SW2_RC_FMT " attempt=%u", SW2_RC_ARGS(rc), g_init_failures);
                return false;
            }

            rc = hiddbgAttachHdlsWorkBuffer(&g_session, g_work_buffer, sizeof(g_work_buffer));
            if (R_FAILED(rc)) {
                ++g_init_failures;
                hiddbgExit();
                SW2_LOG_WARN("[S6][FAIL][HDLS-INIT] hiddbgAttachHdlsWorkBuffer " SW2_RC_FMT " attempt=%u", SW2_RC_ARGS(rc), g_init_failures);
                return false;
            }

            g_hdls_ready = true;
            SW2_LOG_INFO("[S6][OK][HDLS-INIT] hid:dbg work buffer attached session=0x%016llX",
                static_cast<unsigned long long>(g_session.id));
            return true;
        }

        enum class Action : u8 { None, Attach, Detach, SetState };

        void ProcessSlot(size_t index) {
            Action action = Action::None;
            PadKind kind = PadKind::FullKey;
            bluetooth::Address address = {};
            HiddbgHdlsHandle handle = {};
            HiddbgHdlsState state = {};

            {
                std::scoped_lock lk(g_lock);
                Pad &pad = g_pads[index];
                if (pad.attached && !pad.used) {
                    action = Action::Detach;
                } else if (pad.used && !pad.attached && pad.attach_attempts < MaxAttachAttempts) {
                    action = Action::Attach;
                } else if (pad.used && pad.attached && pad.dirty) {
                    action = Action::SetState;
                    pad.dirty = false;
                }
                if (action == Action::None) return;

                pad.busy = true;
                kind = pad.kind;
                address = pad.address;
                handle = pad.handle;
                state = pad.state;
            }

            bool attached_after = action != Action::Attach && action != Action::Detach;
            HiddbgHdlsHandle new_handle = handle;

            switch (action) {
                case Action::Attach: {
                    if (!EnsureHdlsReady()) {
                        std::scoped_lock lk(g_lock);
                        g_pads[index].busy = false;
                        g_pads[index].attach_attempts = MaxAttachAttempts;
                        return;
                    }
                    const HiddbgHdlsDeviceInfo info = MakeDeviceInfo(kind);
                    const Result rc = hiddbgAttachHdlsVirtualDevice(&new_handle, &info);
                    if (R_FAILED(rc)) {
                        SW2_LOG_WARN("[S6][FAIL][HDLS-ATTACH] type=%s mac=%02X:%02X:%02X:%02X:%02X:%02X " SW2_RC_FMT,
                            PadKindName(kind),
                            address.address[0], address.address[1], address.address[2],
                            address.address[3], address.address[4], address.address[5], SW2_RC_ARGS(rc));
                        {
                            std::scoped_lock lk(g_lock);
                            g_pads[index].busy = false;
                            ++g_pads[index].attach_attempts;
                        }
                        os::SleepThread(ams::TimeSpan::FromMilliSeconds(200));
                        return;
                    }
                    // Push the latest decoded state immediately so the new npad is not idle.
                    const Result rc_state = hiddbgSetHdlsState(new_handle, &state);
                    SW2_LOG_INFO("[S6][OK][HDLS-ATTACH] type=%s handle=0x%016llX mac=%02X:%02X:%02X:%02X:%02X:%02X first_state " SW2_RC_FMT
                        " (Horizon should now list the controller; press L+R in Change Grip/Order)",
                        PadKindName(kind), static_cast<unsigned long long>(new_handle.handle),
                        address.address[0], address.address[1], address.address[2],
                        address.address[3], address.address[4], address.address[5], SW2_RC_ARGS(rc_state));
                    attached_after = true;
                    break;
                }
                case Action::Detach: {
                    const Result rc = hiddbgDetachHdlsVirtualDevice(handle);
                    SW2_LOG_INFO("[S6][HDLS-DETACH] type=%s handle=0x%016llX " SW2_RC_FMT,
                        PadKindName(kind), static_cast<unsigned long long>(handle.handle), SW2_RC_ARGS(rc));
                    attached_after = false;
                    break;
                }
                case Action::SetState: {
                    const Result rc = hiddbgSetHdlsState(handle, &state);
                    if (R_FAILED(rc)) {
                        bool still_attached = true;
                        if (R_SUCCEEDED(hiddbgIsHdlsVirtualDeviceAttached(g_session, handle, &still_attached)) && !still_attached) {
                            // hid dropped the device (e.g. sleep); re-attach on the next pass.
                            attached_after = false;
                        }
                        bool should_log = false;
                        {
                            std::scoped_lock lk(g_lock);
                            should_log = ++g_pads[index].set_failures <= MaxSetFailureLogs;
                        }
                        if (should_log) {
                            SW2_LOG_WARN("[S6][FAIL][HDLS-STATE] type=%s handle=0x%016llX still_attached=%u " SW2_RC_FMT,
                                PadKindName(kind), static_cast<unsigned long long>(handle.handle), still_attached ? 1u : 0u, SW2_RC_ARGS(rc));
                        }
                    }
                    break;
                }
                default:
                    break;
            }

            std::scoped_lock lk(g_lock);
            Pad &pad = g_pads[index];
            pad.attached = attached_after;
            pad.handle = new_handle;
            pad.busy = false;
            if (action == Action::Attach) {
                pad.attach_attempts = 0;
            }
        }

        bool HasPendingWork() {
            std::scoped_lock lk(g_lock);
            for (const auto &pad : g_pads) {
                if ((pad.attached && !pad.used) ||
                    (pad.used && !pad.attached && pad.attach_attempts < MaxAttachAttempts) ||
                    (pad.used && pad.attached && pad.dirty)) {
                    return true;
                }
            }
            return false;
        }

        void WorkerThreadFunc(void *) {
            for (;;) {
                g_event.Wait();
                do {
                    for (size_t i = 0; i < MaxPads; ++i) {
                        ProcessSlot(i);
                    }
                } while (HasPendingWork());
            }
        }

        // Caller must hold g_lock.
        Pad *FindOwnedPad(const void *owner) {
            for (auto &pad : g_pads) {
                if (pad.used && pad.owner == owner) return &pad;
            }
            return nullptr;
        }

        // Caller must hold g_lock.
        void StartWorkerIfNeeded() {
            if (g_worker_started) return;
            R_ABORT_UNLESS(os::CreateThread(&g_worker_thread, WorkerThreadFunc, nullptr,
                g_worker_thread_stack, WorkerThreadStackSize, WorkerThreadPriority));
            os::SetThreadNamePointer(&g_worker_thread, "mc::Sw2HdlsWorker");
            os::StartThread(&g_worker_thread);
            g_worker_started = true;
        }

        HiddbgHdlsState NeutralState() {
            HiddbgHdlsState state = {};
            state.battery_level = 4;
            state.flags = BIT(0);
            return state;
        }

    }

    void Attach(const void *owner, const bluetooth::Address &address, PadKind kind) {
        bool full = false;
        {
            std::scoped_lock lk(g_lock);
            StartWorkerIfNeeded();

            Pad *pad = FindOwnedPad(owner);
            if (pad == nullptr) {
                for (auto &candidate : g_pads) {
                    if (!candidate.used && !candidate.attached && !candidate.busy) {
                        pad = &candidate;
                        break;
                    }
                }
            }

            if (pad == nullptr) {
                full = true;
            } else {
                pad->owner = owner;
                pad->address = address;
                pad->kind = kind;
                pad->used = true;
                pad->dirty = true;
                pad->attach_attempts = 0;
                pad->set_failures = 0;
                pad->state = NeutralState();
            }
        }

        if (full) {
            SW2_LOG_WARN("[S6][FAIL][HDLS-QUEUE] all %u virtual pad slots in use", static_cast<u32>(MaxPads));
            return;
        }

        SW2_LOG_INFO("[S6][HDLS-QUEUE] type=%s mac=%02X:%02X:%02X:%02X:%02X:%02X",
            PadKindName(kind),
            address.address[0], address.address[1], address.address[2],
            address.address[3], address.address[4], address.address[5]);
        g_event.Signal();
    }

    void Update(const void *owner, const HiddbgHdlsState &state) {
        {
            std::scoped_lock lk(g_lock);
            Pad *pad = FindOwnedPad(owner);
            if (pad == nullptr) return;
            if (std::memcmp(&pad->state, &state, sizeof(state)) == 0) return;
            pad->state = state;
            pad->dirty = true;
        }
        g_event.Signal();
    }

    void Detach(const void *owner) {
        {
            std::scoped_lock lk(g_lock);
            Pad *pad = FindOwnedPad(owner);
            if (pad == nullptr) return;
            pad->used = false;
            pad->owner = nullptr;
            pad->dirty = false;
        }
        g_event.Signal();
    }

}

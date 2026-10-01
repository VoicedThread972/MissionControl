# Switch 2 development build — 2026-10-01

## Release status

**This is NOT a finished Joy-Con 2 support release.** It is a repaired, opt-in
GATT development build. Successful compilation, a GATT connection, command ACKs,
or player LEDs do not mean Horizon has registered a usable gamepad.
No console/controller hardware validation was performed for these changes.

The requested end-to-end support remains incomplete. In particular, this build
does not make Joy-Con 2 controllers appear in games by itself.

## Firmware baseline

The fork has merged official MissionControl v0.15.2 (`d3941d4`, upstream
`master`, firmware 22.5.0) on top of the v0.15.1 base:

- HID patch: `C3030310E47B3841417518902AFFB2301F0033FC.ips`.
- Atmosphere-libs: `d3083af1827cd6ca2a96feb9316eb85cd01bae1f`.
- libnx: `dbcc1beafc6b47b5ffbeb8ba82463a7d45da40bb`.
- On 22.5.0, upstream requires Atmosphere **1.11.2 or newer**.

This describes the source/patch baseline, not hardware certification of this
fork. No blanket promise is made for "22.0.0+" or unlisted future versions.
Atmosphere 1.12.0 supporting 23.0.0 does **not** establish MissionControl 23.0.0
compatibility. Do not use this package as a 23.0.0 support claim.

23.0.0 status (checked against upstream on merge): **not supported**.

- No upstream MissionControl release or branch contains 23.0.0 patches; the
  latest release is v0.15.2 (22.5.0). Upstream issue #1150 reports that
  MissionControl does not work on 23.0.0.
- The pinned Atmosphere-libs has no `TargetFirmware_23_0_0`; newer
  Atmosphere-libs `master` does. A community report on #1150 says a libs-only
  update boots on 23.0.0 but rumble breaks, which suggests the exefs IPS
  patches need updating.
- New IPS patches require the 23.0.0 bluetooth/btm/hid module binaries and
  cannot be derived from this repository.

Sources checked:

- https://github.com/ndeadly/MissionControl/releases/tag/v0.15.2
- https://github.com/ndeadly/MissionControl/compare/bb54b87...v0.15.2
- https://github.com/Atmosphere-NX/Atmosphere/releases/tag/1.12.0
- Local `documentation/bluetooth_interface.md`, `commands.md`, `hid_reports.md`
  in the parent workspace, and the checked-in libnx btdrv/btm headers.

## Repairs in this build

- GATT discovery/subscription and controller initialization run on the dedicated
  worker, not the shared Bluetooth event thread that must receive their replies.
- Uses discovered service/characteristic/descriptor IDs; requests managed data
  paths, notification registration and CCC writes. Subscribes only to the
  model-specific default input characteristic, avoiding competing streams.
- Validates notification result/type/length and UUID size before copying data.
  Restores report IDs from characteristics, not payload length. All model
  defaults can be 63 bytes; they must not be mistaken for universal report 05.
- Serializes command writes and arms response matching before writing. Rejects
  malformed/unmatched responses; timeout, disconnect and non-78 ACKs fail instead
  of reporting success. Input routing waits for successful initialization.
- Tracks initialization generations, rejects duplicate handlers, removes failed
  Switch 2 handlers, and never falls back to classic HID writes when disconnected.
- Removes trace-replayed 0x15 keys and pairing finalization. The old sequence did
  not verify the cryptographic response or persist host keys. No replacement
  bonding is claimed. GATT input does not require this optional custom pairing.
- File-only S1–S4 diagnostics, locked static formatting buffers, bounded writes.
  No GDB logger. SD I/O remains synchronous/best-effort, not real-time guaranteed.
- Names the existing SL/SR bits without changing the three-byte Switch button
  layout. Adds packed-stick and protocol regression tests.
- Bounds discovery/connection queues, includes full firmware patches and boot
  flag in distribution, and explicitly labels artifacts experimental.

## Remaining implementation blockers

1. **Horizon registration:** current btm MITM only renames real device lists.
   It does not synthesize Switch 2 connected-device entries/change events. A
   safe HID open/close event queue and related btdrv control interception are
   absent. The old `SignalFakeEvent` helper is not safe for concurrent producers
   and is deliberately NOT used as a shortcut. Unknown BTM structure fields
   must not be filled by unsupported guesses.
2. **Scan acquisition:** the bridge passively observes btm-managed scans. Those
   filters may never expose company 0553 advertisements. A supported, scoped
   pairing-screen scan/filter lifecycle is still required. This build does not
   disable/restart global filters (previous experiments correlated with crashes).
3. **GATT hardware sequencing:** verify service cache timing, UUID representation,
   CCC write completion and the ordering/meaning of driver events on target
   firmware. IPC success alone is not proof that a remote subscription completed.
4. **Reconnect and pairing:** controller/host key persistence, AES response
   verification and link-encryption integration are not implemented. Do not use
   SMP pairing: the documented controller protocol does not support it.
5. **Controller behavior:** combined L+R presentation, calibrated motion, normal
   rumble output, wake and sleep/reconnect behavior are not implemented/validated.
   Playing a bootstrap vibration sample is not rumble support.

## Build and package

Use the devkitPro/devkitA64 environment with the above submodules checked out.
From the MissionControl directory:

- `make test`: host C++17 regression tests; optional `HOST_CXX` override.
- `make -j4 dist`: ARM64 sysmodule plus complete distribution.

The resulting ZIP is under `dist/` and is named
`MissionControl-0.15.2-sw2-experimental-<commit>[-dirty].zip`.
`dist/SHA256SUMS` covers files inside the package (not the enclosing ZIP).
The dirty marker and ZIP hash should accompany every hardware test report.
Tests cover protocol validators, ACK matching/first-response retention and stick
packing. They do not simulate Horizon, validate complete button mappings or
prove Bluetooth interoperability.

## Installation and rollback

1. Back up existing MissionControl content, configuration and exefs patches.
   Power the console off before moving its SD card.
2. Extract the complete ZIP at the SD root. Preserve existing live configuration;
   the package supplies a `.template`, not an overriding live `.ini`.
3. Verify the package includes `atmosphere/contents/010000000000bd00/exefs.nsp`,
   `mitm.lst`, `flags/boot2.flag`, and **all** `atmosphere/exefs_patches` directories.
   Copying only the NSP is insufficient.
4. Reboot. Ordinary controller regression tests should be done before enabling
   the experimental path.
5. For deliberate GATT diagnostics only, set
   `enable_switch2_experimental=true` in the `[bluetooth]` section of the live
   `config/MissionControl/missioncontrol.ini`, then reboot. Default is false.
6. Diagnostics are written only to `config/MissionControl/switch2_debug.log`.
   The log is recreated each boot: copy it before the next reboot. Check the build
   signature and fresh sequence numbers; do not interpret an older session.

The parent workspace's deployment helper now requires an explicit SD drive and
copies the complete distribution. It is not automatically executed by the build.
To roll back, power off and restore the backed-up module and matching patches;
alternatively remove the module's `flags/boot2.flag` to disable its startup.

## Reading the diagnostic log

`config/MissionControl/switch2_debug.log` (format version 3) contains only lines
tagged with a protocol stage. Every line has a sequence number and system tick.

| Stage | Covers | Key lines |
|---|---|---|
| S0 | session/startup/config | `[S0][START] build= hos=`, `[S0][CONFIG] enable_switch2_experimental=` |
| S1 | BLE scan and other BLE events | `SCAN-MAC` (first 32), `SCAN-SUMMARY`, `BLE-EVENT ClientConfigureMtu mtu=` |
| S2 | advertisement identification | `[OK][FOUND-nn] type=`, `[SKIP][ADV-0553]` (Nintendo data, unknown layout/PID) |
| S3 | connect and GATT setup | `CONNECT-REQUEST`, `CONNECTED`, `GATT-SERVICE-LOOKUP`, `GATT-<RESP/INPUT>-<CHAR/CCC-DESC/NOTIFY-REG/CCC-WRITE>`, `HANDLER-SELECT`, `GATT-SETUP`, `DISCONNECTED` |
| S4 | command bootstrap | `PAIR-nn`, `CMD-TX` (hex), `CMD-RX` (hex), `CMD-ACK ack= elapsed_ms=`, `DROP][CMD-RX result=` |
| S5 | input | `INPUT-FIRST` (hex), `INPUT-CHANGE buttons=`, `DROP][<reason>`, `STATS` |

Result tags: `[OK]`, `[FAIL]`, `[DROP]` (packet discarded), `[SKIP]` (expected,
not an error), `[STATS]` (per-connection counters written on disconnect).
Result codes are printed as `rc=0xXXXXXXXX(name)`; module 0x123 names are
`stale-or-disconnected`, `ack-timeout`, `ack-rejected`, `service-not-found`, etc.

How to locate the failure:

1. Check `[S0][CONFIG]`. If the experimental path is off, nothing else appears.
2. Find the **first** `[FAIL]` line; later failures are usually consequences.
   `[S3][FAIL][GATT-SETUP]` is a summary; the step-specific `[FAIL]` precedes it.
3. If there is no `[FAIL]`, the last stage reached is the failure point:
   no S1 = no scan results; S1 without S2 = advertisement not matched (look for
   `ADV-0553`); S2 without `CONNECTED` = connect not completed; `CMD-TX` without
   `CMD-RX` = controller did not answer (`ack-timeout`); `CMD-RX` with
   `[DROP]` = response did not match the awaited command.
4. `[S5][OK][INPUT-FIRST]` proves that a notification was decoded and written to
   the HID event buffer. It does **not** prove Horizon accepts the controller
   (device registration is not implemented).
5. `[S5][STATS]` on disconnect: `notifications` vs. `delivered` and the per-reason
   drop counters show where input packets were lost. A `CCC-WRITE` OK only means
   the request was queued; the first notification proves delivery.

Attach the complete log, the ZIP name and its SHA-256 to every report.

## Hardware acceptance checklist (all pending)

- Record exact HOS, Atmosphere, Joy-Con firmware, ZIP SHA-256 and tested L/R model.
- Confirm safe boot and unchanged existing classic-Bluetooth controller behavior
  with experimental mode disabled, then enabled.
- Open Controllers > Change Grip/Order; hold the controller SYNC button until
  LEDs cycle. Log S1 advertisements, S2 model/MAC, S3 connection/GATT and S4 setup result.
  No S1/S2 results means scan acquisition is unresolved, not a parser failure.
- Confirm discovered service/characteristic instances and successful CCC writes;
  verify every bootstrap reply and actual default-format notifications.
- Exercise malformed/truncated reports, missing/wrong ACK, disconnect during each
  initialization phase, rapid reconnect and both Joy-Cons concurrently.
- Implement and then verify Horizon device registration/removal, controls in a
  game, independent/combined Joy-Con behavior and ordinary controller coexistence.
- Validate sleep/wake, reconnect, power-off, battery state and repeated cold boots.
  Do not label the build supported until these checks and the missing features
  have been addressed.
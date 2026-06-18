# Hardware Inventory

Single source of truth for Raspberry Pi devices in the NY&E layout control system — hostname, role, network, and login details. Update this table whenever a device is provisioned, re-imaged, or reassigned.

## Convention

- **Hostname:** `rpi{model}-{role}` (e.g. `rpi5-2`, `rpi3-yard`). Bare labels like `RPi3-0` refer to unprovisioned hardware sitting in a rack, not yet assigned a role.
- **Auth:** SSH key-only for every provisioned RPi. No password SSH login.
- **SSH key:** dedicated key pair per device at `~/.ssh/id_<hostname>` (underscores), with a matching `Host` alias in `~/.ssh/config` (`IdentitiesOnly yes`).
- **Username:** `abyrne` for server-class devices (full LAN access, passwordless sudo); `pi` for kiosk/terminal devices flashed via Raspberry Pi Imager (Imager's default user-creation flow). **Exception:** `rpi3-yard` — Imager's advanced-settings customization didn't apply on the actual flash (settings dialog wasn't filled in before writing the image), so the device went through the on-device first-run wizard and ended up with a user named `abyrne` instead. A `pi` user was created manually after the fact to match this convention; see `YARDMASTER_DESIGN.md` §9.1 for the exact recovery steps. The leftover `abyrne` account on rpi3-yard has no SSH access (password auth disabled, no key installed) — console-only if ever needed.

## Provisioned RPis

| Hostname | Model | Role | Network | User | Auth | SSH Key | Status |
|---|---|---|---|---|---|---|---|
| `rpi5-2` | RPi5 | Central layout server (MQTT broker, fast clock, dispatcher web app, JMRI) | eth0 `192.168.86.36` (home LAN, DHCP reservation) / wlan0 `192.168.10.1` (AP, SSID `NYE_Layout`) | `abyrne` | Key-only, passwordless sudo | `~/.ssh/id_rpi5` | Active |
| `rpi3-yard` | RPi3 B+ | Yardmaster terminal (kiosk, touchscreen) | eth0 `192.168.86.34` (home LAN, maintenance fallback) / wlan0 `192.168.10.20` (DHCP reservation on rpi5-2, `NYE_Layout` — open network, no password) | `pi` (passwordless sudo) | Key-only | `~/.ssh/id_rpi3_yard` | Software provisioning complete (2026-06-18), kiosk dashboard visually confirmed rendering on an HDMI monitor. OS is Debian 13 "trixie" w/ labwc, not Bookworm/LXDE — see `YARDMASTER_DESIGN.md` §9.3/§9.8 for the Chromium GPU-crash and keyring-prompt fixes required on this OS version. **⚠️ OPEN ISSUE: intermittent power under-voltage** (`vcgencmd get_throttled` showed live throttling even after one PSU swap) — see §9.9, must be confirmed resolved (`get_throttled` = `0x0`) before treating this device as reliable. Touch/display verification with the real ELECROW screen still pending (not yet physically attached — CAD enclosure test print in progress). |

## Unprovisioned hardware (rack inventory)

| Label | Model | Notes | Location |
|---|---|---|---|
| RPi3-0 | RPi3 | GPIO breakout board connected to breadboard | Office Desk Rack |
| RPi3-1 | RPi3 | Standard, ready | Office Desk Rack |
| RPi3-2 | RPi3 | Camera connected | Office Desk Rack |
| RPi3-3 | RPi3 | Standard, ready | Office Desk Rack |
| RPi4-1 | RPi4 | Fully connected | Office Desk Rack |
| 2× SPROG RPis | — | OS SD cards installed, dedicated to SPROG/DCC bridge | Prusa Frame Rack |

## See also

- `docs/IMPLEMENTATION_PLAN.md` — rpi5-2 deployment and server setup
- `docs/YARDMASTER_DESIGN.md` §9 — rpi3-yard provisioning steps (OS image, kiosk autostart, DHCP reservation)

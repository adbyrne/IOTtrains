# TO_Signal — Train Order Signal Controller Firmware

> **Superseded for CYD stations (2026-05-23):** As of Station_OS v2.3.0, TO signal arm servos are driven by the station CYD unit via a PCA9685 on the CN1 I2C connector (IO27=SDA, IO22=SCL). No separate TO_Signal ESP32 is deployed at stations with CYD units. This firmware remains the reference design for any non-CYD installation.

ESP32 firmware controlling two TO signal arms (N=northbound, S=southbound) per station.
Each arm is driven by a dedicated servo; arms are commanded independently by the dispatcher.

## Hardware

- **MCU:** ESP32 bare dev board
- **Servo N (northbound arm):** GPIO 13
- **Servo S (southbound arm):** GPIO 14
- **Physical design:** `CADlayout/Servo/` — two-servo bracket with Bowden cable linkage

## TO-Signal Stations

| Station | ID |
|---------|----|
| Xina Pass | XP |
| Becs Bend | BB |
| Jacks Creek | JC |
| Michelles Cove | MC |
| Stans Knob | SK |

WP and HC (terminus stations) do not have TO signal arms.

## MQTT Topics

| Topic | Dir | QoS | Retained | Payload |
|-------|-----|-----|----------|---------|
| `trains/signal/{id}/to/{N\|S}/cmd`   | Sub | 1 | Yes | `{"state": "raised"}` or `{"state": "lowered"}` |
| `trains/signal/{id}/to/{N\|S}/state` | Pub | 1 | Yes | `{"state": "raised", "station_id": "BB", "dir": "N", "rr_time": "10:30"}` |

Both cmd topics are retained — the controller restores commanded state on reconnect by
subscribing at connect time and processing the replayed retained messages.

Also subscribes to `trains/clock/time` (QoS 0) to track railroad time for state reports.

## Configuration

All config is stored in NVS. Set via serial CLI (see Calibration below).

| Setting | CLI key | Description |
|---------|---------|-------------|
| WiFi SSID | `set ssid` | Network name |
| WiFi password | `set pass` | |
| MQTT broker IP | `set mqtt` | e.g. `192.168.10.1` |
| Station ID | `set station` | `XP`, `BB`, `JC`, `MC`, or `SK` |
| MQTT username | `set muser` | default: `to_signal` |
| MQTT password | `set mpas` | |
| N arm raised angle | `set angle N raised` | degrees (0–180) |
| N arm lowered angle | `set angle N lowered` | degrees (0–180) |
| S arm raised angle | `set angle S raised` | degrees (0–180) |
| S arm lowered angle | `set angle S lowered` | degrees (0–180) |

## Calibration

Connect via serial monitor at 115200 baud. On first boot (unprovision), the CLI prompt
appears automatically. On normal boots, type any command to interact.

```
=== TO_Signal: not configured ===
Type 'help' for commands.
```

**Typical first-boot flow:**

```
set station BB
set ssid NYE_Layout
set pass <password>
set mqtt 192.168.10.1
sweep N 45          # move N arm, find raised angle
set angle N raised 45
sweep N 90          # find lowered angle
set angle N lowered 90
raise N             # confirm with bounce
lower N
# repeat for S arm
save
restart
```

**All CLI commands:** `help` prints the full list.

## Servo Motion

- Sweep speed: ~15 ms/degree (~0.9 s for a 60° sweep)
- Each move ends with a brief mechanical bounce: slight overshoot → settle, mimicking
  the Bowden cable linkage stopping
- Each station may have different travel angles — calibrate each unit individually

## Build

```bash
cd TO_Signal
pio run
pio run --target upload
pio device monitor
```

## Architecture

- Two FreeRTOS tasks: `servo` (priority 5) and `cli` (priority 1)
- All servo writes serialized through a FreeRTOS queue — CLI and MQTT callbacks
  both post to the queue; the servo task executes all moves
- WiFi/MQTT reconnect via FreeRTOS software timers (same pattern as Switch_Control)
- State fully recoverable from MQTT retained messages on restart

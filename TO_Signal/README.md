# TO_Signal — Train Order Signal Controller Firmware

ESP32 firmware for controlling the TrainOrderServo seesaw signal arm at TO-signal-capable stations.

## Hardware

- **MCU:** ESP32 (bare dev board — no display needed)
- **Actuator:** Servo motor via direct GPIO (or PCA9685 if sharing with other servos)
- **Physical design:** SwitchToggle/TrainOrderServo seesaw arm (see `CADlayout/TrainOrderServo/`)

## TO-Signal Stations

| Station | ID | Signal |
|---------|----|--------|
| Xina Pass | XP | Yes |
| Becs Bend | BB | Yes |
| Jacks Creek | JC | Yes |
| Michelles Cove | MC | Yes |
| Stans Knob | SK | Yes |

WP and HC (terminus stations) do not have TO signal arms.

## MQTT Topics

| Topic | Direction | Payload |
|-------|-----------|---------|
| `trains/signal/{station_id}/to/cmd` | Subscribe | `{"state": "raised"}` or `{"state": "lowered"}` |
| `trains/signal/{station_id}/to/state` | Publish | `{"state": "raised", "station_id": "BB", "rr_time": "10:30"}` |

The command topic is **retained** — the controller restores commanded state on reconnect.

## Configuration (NVS, set via serial)

- WiFi SSID + password
- MQTT broker IP
- Station ID (XP / BB / JC / MC / SK)
- Servo angle for RAISED position
- Servo angle for LOWERED position

## Build

```bash
cd TO_Signal && pio run
pio run --target upload
pio device monitor
```

## Implementation Notes

- Based on Switch_Control connection/reconnect pattern (FreeRTOS timers, AsyncMqttClient).
- Servo movement should be slow and deliberate (same as Switch_Control).
- On MQTT reconnect, subscribe to retained cmd topic to restore last commanded state.
- Publish state report after each movement completes.

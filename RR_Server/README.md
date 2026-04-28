# RR_Server — RPi5 Layout Control Server

Python-based server software for the NY&E Northern Lights Subdivision layout control system. Runs on the dedicated RPi5 alongside Mosquitto and JMRI.

## Planned Components

```
RR_Server/
  mosquitto/
    mosquitto.conf        ← broker config (auth, persistence, logging)
  fast_clock/
    clock_service.py      ← fast clock daemon; publishes trains/clock/time
    clock_state.json      ← persisted clock state (survives restart)
  dispatcher/
    app.py                ← FastAPI web application (dispatcher dashboard)
    mqtt_client.py        ← MQTT bridge (paho-mqtt → WebSocket)
    static/               ← CSS, JS
    templates/            ← Jinja2 HTML templates
  scripts/
    setup_ap.sh           ← configure hostapd + dnsmasq on RPi5
    setup_mosquitto.sh    ← install and configure broker
    setup_jmri.sh         ← JMRI install + LocoNet config notes
    install_services.sh   ← systemd unit files for all services
  requirements.txt
  README.md
```

## Services (systemd)

| Service | Description |
|---------|-------------|
| `mosquitto` | MQTT broker |
| `rr-clock` | Fast clock daemon |
| `rr-dispatcher` | Dispatcher web application |
| `jmri` | JMRI (DCC / WiThrottle) |

All services start at boot. Clock service depends on Mosquitto being ready.

## Network Configuration (RPi5)

- **WiFi interface (wlan0):** hostapd AP — SSID `NYE_Layout`, static IP `192.168.10.1`
- **DHCP (dnsmasq):** serves `192.168.10.10` – `192.168.10.99`
- **Ethernet (eth0):** home LAN (DHCP, for maintenance/SSH) — optional
- **Mosquitto:** binds to `192.168.10.1:1883` (layout WiFi only; not exposed to home LAN)

## Dispatcher Web App Access

Browser on RPi5 Display 1: `http://localhost:5000`
Remote access (maintenance): `http://192.168.10.1:5000` from layout WiFi

## Phase 1 Deliverables

- [ ] hostapd + dnsmasq configuration and setup script
- [ ] Mosquitto config (auth, persistence, ACL)
- [ ] Fast clock service (publish `trains/clock/time`, handle `trains/clock/control`)
- [ ] Dispatcher UI: clock display with pause/start/set controls
- [ ] Dispatcher UI: station online/offline status panel
- [ ] systemd unit files for all services

## Dependencies

```
fastapi
uvicorn
paho-mqtt
jinja2
python-dotenv
```

"""In-memory state store and WebSocket connection manager."""

from dataclasses import dataclass, field
from fastapi import WebSocket

STATION_IDS = ["WP", "XP", "BB", "JC", "MC", "SK", "HC"]

# Stations with physical TO signal arms
TO_SIGNAL_STATIONS = {"XP", "BB", "JC", "MC", "SK"}

# WP-XP block section boundary signals — distinct from TO signals (order
# delivery) and station-specific (one per station, not N/S pairs): WP's
# signal gates northbound entry into the block; XP's gates southbound entry.
# Momentary: normally "lowered" (stop); a dispatcher trigger pulses to
# "raised" (clear) for BLOCK_SIGNAL_PULSE_SECONDS, then auto-reverts.
BLOCK_SIGNAL_STATIONS = {"WP", "XP"}

STATION_NAMES = {
    "WP": "Williamsport",
    "XP": "Xina Pass",
    "BB": "Becs Bend",
    "JC": "Jacks Creek",
    "MC": "Michelles Cove",
    "SK": "Stans Knob",
    "HC": "Hemlock Crest",
}

OS_LOG_MAX = 50
TO_LOG_MAX = 100
YARD_NOTIF_MAX = 20


@dataclass
class AppState:
    clock: dict = field(default_factory=dict)
    stations: dict = field(default_factory=dict)    # station_id -> status dict
    to_signals: dict = field(default_factory=dict)  # station_id -> {"N": state, "S": state}
    block_signals: dict = field(default_factory=dict)  # station_id -> "raised"|"lowered" (WP, XP only)
    block_signal_tasks: dict = field(default_factory=dict)  # station_id -> asyncio.Task (pulse auto-revert)
    os_log: list = field(default_factory=list)      # newest first, capped at OS_LOG_MAX
    to_log: list = field(default_factory=list)      # newest first, capped at TO_LOG_MAX
    to_seq: int = 0
    current_day: int = 0                            # tracks RR day for Rule 203 reset
    consists: dict = field(default_factory=dict)         # train/engine number -> consist dict
    yard_notifications: list = field(default_factory=list)  # newest first, capped at YARD_NOTIF_MAX
    yard_tracks: list = field(default_factory=list)         # loaded from yard.json at startup
    roster: dict = field(default_factory=dict)               # {"engines": [...], "cabooses": [...]}, loaded from roster.json at startup
    equipment_status: dict = field(default_factory=dict)    # {"engines": {rn: {status, rr_time, day}}, "cabooses": {...}}
    extra_seq: int = 0                                      # placeholder "XTRA{n}" id counter
    _clients: set = field(default_factory=set)

    def next_seq(self) -> int:
        self.to_seq += 1
        return self.to_seq

    def next_extra_seq(self) -> int:
        self.extra_seq += 1
        return self.extra_seq

    def reset_seq(self) -> None:
        """Reset TO sequence to 0 so the next order is No. 1 (Rule 203)."""
        self.to_seq = 0

    def check_day(self, new_day: int) -> bool:
        """Update current day; reset TO seq if day changed. Returns True if reset occurred."""
        if new_day != self.current_day:
            self.current_day = new_day
            self.to_seq = 0
            return True
        return False

    def record_to(self, entry: dict) -> None:
        """Prepend a newly-issued TO entry with per-station and per-train ACK tracking."""
        entry["acks"] = {sid: None for sid in entry.get("addressed_to", [])}
        entry["train_acks"] = {t: None for t in entry.get("trains", [])}
        self.to_log.insert(0, entry)
        if len(self.to_log) > TO_LOG_MAX:
            self.to_log = self.to_log[:TO_LOG_MAX]

    def record_ack(self, seq: int, station_id: str, ack_data: dict) -> dict | None:
        """Record an ACK for a station; return the TO entry if found, else None."""
        for entry in self.to_log:
            if entry.get("seq") == seq:
                entry["acks"][station_id] = ack_data
                return entry
        return None

    def record_train_rcvd(self, seq: int, train: str, rr_time: str) -> dict | None:
        """Record per-train receipt; return the TO entry if found, else None."""
        for entry in self.to_log:
            if entry.get("seq") == seq:
                if train in entry.get("train_acks", {}):
                    entry["train_acks"][train] = rr_time
                return entry
        return None

    def record_consist(self, train: str, payload: dict) -> dict:
        """Upsert a consist record, deriving cars_total when loads/empties are both present."""
        entry = dict(payload)
        entry["train"] = train
        if entry.get("cars_loaded") is not None and entry.get("cars_empty") is not None:
            entry["cars_total"] = entry["cars_loaded"] + entry["cars_empty"]
        entry["rr_time"] = f"{self.clock.get('hour', 0):02d}:{self.clock.get('minute', 0):02d}"
        entry["day"] = self.clock.get("day")
        self.consists[train] = entry
        return entry

    def record_notification(self, entry: dict) -> None:
        """Prepend a yard notification, newest first, capped at YARD_NOTIF_MAX."""
        self.yard_notifications.insert(0, entry)
        if len(self.yard_notifications) > YARD_NOTIF_MAX:
            self.yard_notifications = self.yard_notifications[:YARD_NOTIF_MAX]

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._clients.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self._clients.discard(ws)

    async def broadcast(self, event: dict) -> None:
        dead = set()
        for ws in self._clients:
            try:
                await ws.send_json(event)
            except Exception:
                dead.add(ws)
        for ws in dead:
            self._clients.discard(ws)

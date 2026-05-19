"""In-memory state store and WebSocket connection manager."""

from dataclasses import dataclass, field
from fastapi import WebSocket

STATION_IDS = ["WP", "XP", "BB", "JC", "MC", "SK", "HC"]

# Stations with physical TO signal arms
TO_SIGNAL_STATIONS = {"XP", "BB", "JC", "MC", "SK"}

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


@dataclass
class AppState:
    clock: dict = field(default_factory=dict)
    stations: dict = field(default_factory=dict)    # station_id -> status dict
    to_signals: dict = field(default_factory=dict)  # station_id -> {"N": state, "S": state}
    os_log: list = field(default_factory=list)      # newest first, capped at OS_LOG_MAX
    to_log: list = field(default_factory=list)      # newest first, capped at TO_LOG_MAX
    to_seq: int = 0
    current_day: int = 0                            # tracks RR day for Rule 203 reset
    _clients: set = field(default_factory=set)

    def next_seq(self) -> int:
        self.to_seq += 1
        return self.to_seq

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
        """Prepend a newly-issued TO entry with per-station ACK tracking."""
        entry["acks"] = {sid: None for sid in entry.get("addressed_to", [])}
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

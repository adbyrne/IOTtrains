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


@dataclass
class AppState:
    clock: dict = field(default_factory=dict)
    stations: dict = field(default_factory=dict)   # station_id -> status dict
    to_signals: dict = field(default_factory=dict) # station_id -> {"N": state, "S": state}
    _clients: set = field(default_factory=set)

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

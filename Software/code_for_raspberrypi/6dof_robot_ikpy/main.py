"""
FastAPI Realtime-Dashboard
==========================

Architektur:
- /ws/dashboard  -> Browser-Dashboards verbinden sich hier (Slider senden, Live-Daten empfangen)
- /ws/data       -> Externe Programme verbinden sich hier (Live-Daten senden, Slider-Werte empfangen)

Der Server selbst speichert nichts dauerhaft. Er ist nur ein "Vermittler" (Broadcaster),
der Nachrichten zwischen beiden Seiten weiterleitet.

Starten:
    pip install -r requirements.txt
    uvicorn main:app --reload

Dann im Browser: http://localhost:8000
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pathlib import Path
from typing import List

app = FastAPI()

BASE_DIR = Path(__file__).parent


class ConnectionManager:
    """Verwaltet alle offenen WebSocket-Verbindungen und leitet Nachrichten weiter."""

    def __init__(self) -> None:
        self.dashboard_clients: List[WebSocket] = []
        self.data_clients: List[WebSocket] = []

    async def connect_dashboard(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.dashboard_clients.append(websocket)

    async def connect_data(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.data_clients.append(websocket)

    def disconnect_dashboard(self, websocket: WebSocket) -> None:
        if websocket in self.dashboard_clients:
            self.dashboard_clients.remove(websocket)

    def disconnect_data(self, websocket: WebSocket) -> None:
        if websocket in self.data_clients:
            self.data_clients.remove(websocket)

    async def broadcast_to_dashboards(self, message: dict) -> None:
        """Schickt eine Nachricht an alle offenen Browser-Dashboards."""
        for ws in list(self.dashboard_clients):
            try:
                await ws.send_json(message)
            except Exception:
                self.disconnect_dashboard(ws)

    async def broadcast_to_data_clients(self, message: dict) -> None:
        """Schickt eine Nachricht an alle verbundenen externen Programme."""
        for ws in list(self.data_clients):
            try:
                await ws.send_json(message)
            except Exception:
                self.disconnect_data(ws)


manager = ConnectionManager()


@app.get("/", response_class=HTMLResponse)
def read_root() -> str:
    """Liefert die Dashboard-Seite aus (klassischer GET-Endpoint, wie in deinem Beispiel)."""
    html_path = BASE_DIR / "static" / "dashboard.html"
    return html_path.read_text(encoding="utf-8")


@app.websocket("/ws/dashboard")
async def dashboard_ws(websocket: WebSocket) -> None:
    """
    Verbindung für Browser-Dashboards.
    Eingehend: Slider-Änderungen vom Nutzer -> werden an alle externen Programme weitergeleitet.
    Ausgehend: Live-Daten von externen Programmen -> siehe broadcast_to_dashboards().
    """
    await manager.connect_dashboard(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            await manager.broadcast_to_data_clients(data)
    except WebSocketDisconnect:
        manager.disconnect_dashboard(websocket)


@app.websocket("/ws/data")
async def data_ws(websocket: WebSocket) -> None:
    """
    Verbindung für externe Programme (z. B. ein Sensor-Skript, eine Simulation, etc.).
    Eingehend: Live-Daten vom externen Programm -> werden an alle Dashboards weitergeleitet.
    Ausgehend: Slider-Werte vom Dashboard -> siehe broadcast_to_data_clients().
    """
    await manager.connect_data(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            await manager.broadcast_to_dashboards(data)
    except WebSocketDisconnect:
        manager.disconnect_data(websocket)

# Statische Dateien (falls du z.B. eigene CSS/JS-Dateien ergänzen willst)
app.mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static")
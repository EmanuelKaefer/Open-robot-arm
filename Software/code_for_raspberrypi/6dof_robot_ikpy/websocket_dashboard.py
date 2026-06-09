# websocket_dashboard.py
# Real-time dashboard with WebSocket and channel subscriptions
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from typing import Dict, Set
import asyncio
import json
from datetime import datetime
import asyncio
import psutil
import random
from datetime import datetime

app = FastAPI()

class DashboardManager:
    """Manages WebSocket connections and channel subscriptions"""

    def __init__(self):
        self.connections: Dict[str, WebSocket] = {}  # client_id -> websocket
        self.subscriptions: Dict[str, Set[str]] = {}  # channel -> client_ids
        self.running = True

    async def connect(self, websocket: WebSocket, client_id: str):
        """Accept new connection"""
        await websocket.accept()
        self.connections[client_id] = websocket
        # Subscribe to default channel
        self.subscribe(client_id, "system")

    def disconnect(self, client_id: str):
        """Clean up on disconnect"""
        if client_id in self.connections:
            del self.connections[client_id]
        # Remove from all channel subscriptions
        for subscribers in self.subscriptions.values():
            subscribers.discard(client_id)

    def subscribe(self, client_id: str, channel: str):
        """Add client to channel"""
        if channel not in self.subscriptions:
            self.subscriptions[channel] = set()
        self.subscriptions[channel].add(client_id)

    def unsubscribe(self, client_id: str, channel: str):
        """Remove client from channel"""
        if channel in self.subscriptions:
            self.subscriptions[channel].discard(client_id)

    async def broadcast_to_channel(self, channel: str, data: dict):
        """Send data to all subscribers of a channel"""
        if channel not in self.subscriptions:
            return

        message = json.dumps({"channel": channel, "data": data})
        disconnected = []

        for client_id in self.subscriptions[channel]:
            if client_id in self.connections:
                try:
                    await self.connections[client_id].send_text(message)
                except Exception:
                    disconnected.append(client_id)

        # Clean up disconnected clients
        for client_id in disconnected:
            self.disconnect(client_id)

# Global manager instance
manager = DashboardManager()

@app.websocket("/ws/dashboard/{client_id}")
async def dashboard_websocket(websocket: WebSocket, client_id: str):
    """WebSocket endpoint for dashboard connections"""
    await manager.connect(websocket, client_id)

    try:
        while True:
            # Receive commands from client
            data = await websocket.receive_json()

            # Handle subscription commands
            if data.get("action") == "subscribe":
                channel = data.get("channel")
                manager.subscribe(client_id, channel)
                await websocket.send_json({
                    "status": "subscribed",
                    "channel": channel
                })

            elif data.get("action") == "unsubscribe":
                channel = data.get("channel")
                manager.unsubscribe(client_id, channel)
                await websocket.send_json({
                    "status": "unsubscribed",
                    "channel": channel
                })

    except WebSocketDisconnect:
        manager.disconnect(client_id)

#publisher

async def publish_system_metrics(manager: DashboardManager):
    """Publish system metrics every second"""
    while manager.running:
        metrics = {
            "timestamp": datetime.utcnow().isoformat(),
            "cpu_percent": psutil.cpu_percent(interval=None),
            "memory_used_gb": psutil.virtual_memory().used / (1024**3),
            "memory_total_gb": psutil.virtual_memory().total / (1024**3)
        }
        await manager.broadcast_to_channel("system", metrics)
        await asyncio.sleep(1)

async def publish_application_metrics(manager: DashboardManager):
    """Publish application-specific metrics"""
    # Simulated application metrics
    request_count = 0
    error_count = 0

    while manager.running:
        # In production, these would come from actual counters
        request_count += random.randint(10, 50)
        error_count += random.randint(0, 2)

        metrics = {
            "timestamp": datetime.utcnow().isoformat(),
            "requests_total": request_count,
            "errors_total": error_count,
            "error_rate": error_count / max(request_count, 1) * 100,
            "avg_response_ms": random.uniform(50, 200)
        }
        await manager.broadcast_to_channel("application", metrics)
        await asyncio.sleep(1)

@app.on_event("startup")
async def start_publishers():
    """Start background metric publishers"""
    asyncio.create_task(publish_system_metrics(manager))
    asyncio.create_task(publish_application_metrics(manager))

@app.on_event("shutdown")
async def stop_publishers():
    """Signal publishers to stop"""
    manager.running = False


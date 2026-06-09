# publisher.py
# Background task that publishes metrics to dashboard channels
import asyncio
import psutil
import random
from datetime import datetime

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
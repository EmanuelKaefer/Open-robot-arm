import ikpy.chain
import numpy as np
import json
import websockets
import asyncio

# the URL of the server we comunikate with
SERVER_URL = "ws://localhost:8000/ws/data"

async def listen_for_coordinates(websocket):
    """Hoert auf X/Y/Z-Werte, die vom Dashboard kommen."""
    async for raw_message in websocket:
        data = json.loads(raw_message)
        if data["key"] in ("x", "y", "z"):
            print(f"[Koordinate empfangen] {data['key']} = {data['value']} mm")
    


async def send_status_updates(websocket):
    """Simuliert Statusmeldungen einer Maschine."""
 
    # 1. Erst eine Referenzfahrt (Homing) simulieren
    await websocket.send(json.dumps({"key": "status", "value": "Homing"}))
    await websocket.send(json.dumps({"key": "homing", "value": False}))
    await websocket.send(json.dumps({"key": "error", "value": False}))
    await asyncio.sleep(3)
 
    await websocket.send(json.dumps({"key": "homing", "value": True}))
    await websocket.send(json.dumps({"key": "status", "value": "Idle"}))

async def main():
    async with websockets.connect(SERVER_URL) as websocket:
        print(f"Verbunden mit {SERVER_URL}")
        await asyncio.gather(
            listen_for_coordinates(websocket),
            send_status_updates(websocket),
        )
 
 
if __name__ == "__main__":
    asyncio.run(main())
 

# ── Chain laden ──────────────────────────────────────────────────────
chain = ikpy.chain.Chain.from_urdf_file(
    "robot_armV2SLDASM.urdf",
    active_links_mask=[False, True, True, True, True, True, True]
)

# ── Echte Limits deiner Achsen (anpassen!) ───────────────────────────
LIMITS = [
    (-3.1416, 3.1416),  # J1 Basis
    (-1.5708, 1.5708),  # J2 Shoulder
    (-1.5708, 1.5708),  # J3 Elbow
    (-3.1416, 3.1416),  # J4 Wrist turn
    (-2.0944, 1.5708),  # J5 Wrist tilt
    (-3.1416, 3.1416),  # J6 Gripper turn
]

# ── Greifer-Orientierung: horizontal/gerade ──────────────────────────
GRIPPER_ORIENTATION = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

# ── IK Funktion ──────────────────────────────────────────────────────
last_angles = [0, 0, 0, 0, 0, 0, 0]  # Startet von Nullstellung

def compute_ik(x, y, z):
    global last_angles

    joint_angles = chain.inverse_kinematics(
        target_position=[x, y, z],
        target_orientation=GRIPPER_ORIENTATION,
        orientation_mode="all",
        initial_position=last_angles  # vorherige Position als Startpunkt
    )

    active_angles = joint_angles[1:7]

    # Limit-Check
    for i, (angle, (low, high)) in enumerate(zip(active_angles, LIMITS)):
        if not (low <= angle <= high):
            print(f"WARNUNG: Joint {i+1} außerhalb Limits: {np.degrees(angle):.2f}°")
            return None  # ungültige Lösung — nicht senden

    # FK Verifikation — wie weit ist die echte Position vom Ziel?
    fk = chain.forward_kinematics(joint_angles)
    actual_pos = fk[:3, 3]
    error = np.linalg.norm(actual_pos - np.array([x, y, z]))
    if error > 0.005:  # 5mm Toleranz
        print(f"WARNUNG: Positionsfehler {error*1000:.1f}mm — Lösung ungenau")
        return None

    last_angles = joint_angles  # für nächsten Aufruf speichern

    return {
        "angles_rad": active_angles.tolist(),
        "angles_deg": np.degrees(active_angles).tolist()
    }

# ── Hauptloop ────────────────────────────────────────────────────────
print("IK bereit. Eingabe: x y z (in Metern), oder 'q' zum Beenden")
print("Beispiel: 0.3 0.1 0.4\n")

while True:
    try:
        user_input = input("Zielposition: ").strip()
        if user_input.lower() == 'q':
            break

        x, y, z = map(float, user_input.split())
        result = compute_ik(x, y, z)

        if result is None:
            print("→ Keine gültige Lösung gefunden\n")
            continue

        print("\n─── Joint Winkel ───")
        for i, (rad, deg) in enumerate(zip(result["angles_rad"], result["angles_deg"])):
            print(f"  J{i+1}: {rad:.4f} rad  ({deg:.2f}°)")
        print()

    except ValueError:
        print("Eingabe ungültig — Format: x y z (z.B. 0.3 0.1 0.4)\n")
    except KeyboardInterrupt:
        break





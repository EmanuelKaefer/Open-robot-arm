## Bug: MoveIt Execution schlägt fehl trotz erfolgreichem Planning

### Symptom
- MoveIt Demo (`ros2 launch robot_moveit_config demo.launch.py`) startet ohne offensichtliche Fehlermeldung im Log.
- **Planning funktioniert** einwandfrei.
- **Execution schlägt immer fehl** ("Failed to validate trajectory: couldn't receive full current joint state" / trajectory execution ABORTED).
- Keine `/joint_states` werden published.

### Root Cause
`ros2_control_node` stürzt beim Laden von `arm_controller` (Typ `joint_trajectory_controller/JointTrajectoryController`) mit **SIGSEGV** ab:


Weil dieser Absturz während des `load_controller`-Service-Calls passiert, hängt der `controller_manager` anschließend fest. Dadurch laufen die Spawner für `arm_controller` und `joint_state_broadcaster` in Timeouts und sterben. Ohne aktiven `joint_state_broadcaster` gibt es nie `/joint_states` → MoveIt kann beim Ausführen den aktuellen Robotergelenkzustand nicht validieren → Execution bricht ab.

Planning ist davon nicht betroffen, weil MoveIt dafür keinen Live-Zustand vom echten Roboter/Controller braucht.

### Ursache
Seit **ROS 2 Jazzy** parst der `joint_trajectory_controller` in `on_init()` das URDF, um `continuous`-Gelenke automatisch zu erkennen (für automatisches "Angle Wraparound", damit endlos drehende Gelenke nicht künstlich auf ±π begrenzt werden – siehe [Jazzy Migration Notes](https://github.com/ros-controls/ros2_controllers/blob/master/doc/migration/Jazzy.rst)).

Dieses URDF-Parsing in `on_init()` verursacht bei unserem Roboter-URDF einen Segfault, sobald ein Gelenk vom Typ `continuous` definiert ist.

Betroffen waren in unserem URDF:
- `shoulder_joint1`
- `wrist_turn_joint`
- `gripper_turn_joint`

### Fix
Alle drei Gelenke von `type="continuous"` auf `type="revolute"` mit großzügigen Limits umgestellt (praktisch gleichbedeutend mit "continuous", aber ohne den fehlerhaften Codepfad auszulösen):

```xml
<limit
  lower="-6.28"
  upper="6.28"
  effort="50"
  velocity="3.14" />
```

Nach dem Umbau und `colcon build` lädt `arm_controller` sauber, `/joint_states` werden published, Execution funktioniert.

### Bekannte Einschränkung
Da die Gelenke jetzt technisch `revolute` mit Limit `±2π` sind statt echt `continuous`, ist eine mehrfache Umdrehung über 360° hinaus nicht mehr möglich (praktisch meist irrelevant bei ±6.28 rad Spielraum). Sollte das relevant werden: prüfen, ob ein neueres `ros-jazzy-joint-trajectory-controller`-Paket den zugrunde liegenden Bug behoben hat, dann ggf. zurück auf `continuous` wechseln.

### Debugging-Vorgehen (falls ähnliches Problem nochmal auftritt)
1. `ros2_control_node` isoliert mit `gdb` starten:
```bash
   gdb --args /opt/ros/jazzy/lib/controller_manager/ros2_control_node --ros-args \
     --params-file <pfad-zu>/ros2_controllers.yaml \
     -r /controller_manager/robot_description:=/robot_description
```
2. In gdb: `run`
3. In separatem Terminal den Absturz auslösen: `ros2 run controller_manager spawner arm_controller`
4. Nach Crash in gdb: `bt full` für vollen Backtrace


Jetzt: ros2 control list_controllers
[INFO] [1787095130.856134494] [_ros2cli_77557]: waiting for service /controller_manager/list_controllers to become available...
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
davor war alles timeout.

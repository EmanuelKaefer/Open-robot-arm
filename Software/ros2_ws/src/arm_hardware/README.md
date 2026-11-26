# arm_hardware

A ros2_control hardware interface (`hardware_interface::SystemInterface`) for the robot arm. It sends joint position setpoints to the STM32 driver board over UART and reads back the positions the board has output.

**Status: milestone 5 (drives the arm).** `ArmSystem` is complete apart from homing:

- `on_configure` opens the port and waits for STATE and DRV_OK.
- `on_activate` brings the STM to IDLE: CLEARFAULT if needed, CONFIG, SETZERO (once per configure), ENABLE 1.
- `write()` streams one SETPOINT per cycle. Each axis is limited to the STM vmax per cycle, so a bad command cannot cause a JUMP fault.
- `on_deactivate` and `on_shutdown` brake the arm with ENABLE 0.

Not done yet: MoveIt integration (milestone 6) and homing (milestone 7).

## Data flow

```
RViz -> move_group -> arm_controller (JointTrajectoryController, 100 Hz)
     -> command "<joint>/position" [rad] -> ArmSystem::write() -> SETPOINT [steps] -> UART -> STM
STM  -> STATE every 10 ms -> ArmSystem::read() -> state "<joint>/position", "<joint>/velocity"
     -> joint_state_broadcaster -> /joint_states -> RViz
```

- **Protocol:** `doc/Software/UART-Protokoll.md`
- **Python reference:** `Software/test_codes/Home/robot_proto.py`
- **Source of truth:** the firmware (`Software/driver_board_software/Core/Src/uartconfig.c`, `motion.c`)

## Files

| File | Content |
|---|---|
| `include/arm_hardware/serial_port.hpp`, `src/serial_port.cpp` | POSIX serial port (termios), non-blocking, no ROS |
| `include/arm_hardware/stm_protocol.hpp`, `src/stm_protocol.cpp` | Constants, frame encoding, CRC-16, stream parser, no ROS. A port of `robot_proto.py` |
| `include/arm_hardware/arm_system.hpp`, `src/arm_system.cpp` | The ros2_control plugin: lifecycle, `read()`, `write()` |
| `test/test_stm_protocol.cpp` | gtest cases for the protocol layer (no simulator needed) |
| `test/test_sim_link.cpp` | Integration tests for `SerialPort` and the protocol against the simulator (`/tmp/ttySTM`). They are skipped if the simulator is not running |
| `test/test_arm_system.cpp` | Tests for the plugin itself, driven like the controller_manager does. `ArmSystemInit.*` checks the parameters and always runs. `SimArmSystem.*` runs against the simulator |
| `arm_hardware.xml` | pluginlib registration as `arm_hardware/ArmSystem` |
| `launch/hw_test.launch.py`, `test/bringup/` | Test setup without MoveIt: dummy URDF with the real joint names, `ros2_control_node`, `joint_state_broadcaster`, `arm_controller` (JTC) |
| `scripts/test_trajectory.py` | End-to-end check: sends trajectories to `arm_controller` and compares `/joint_states` with the targets |

## Suggested order (milestones)

1. **Environment.** Attach the board to WSL and check it with `robot_proto.py` (see below).
2. ✅ **`stm_protocol` and its tests.** Done when `colcon test --packages-select arm_hardware` passes.
3. ✅ **`serial_port` and `test_sim_link`.** Start `stm_sim.py`, then run `colcon test`. Done when all `SimLink` tests pass.
4. ✅ **`on_init`, `on_configure` and `read()`, with no sending.** Done when `/joint_states` shows the STM position (see "Read test" below).
5. ✅ **`on_activate` and `write()`.** This step adds CONFIG, SETZERO, ENABLE 1 and streaming SETPOINTs. Done when the `SimArmSystem` tests pass and `test_trajectory.py` prints PASS.
6. **Hook it into `robot_moveit_config`.** See the section below. Then drag the end effector in RViz and run Plan & Execute.
7. **LATER: homing.** Write `start_homing()`, then add the service node in `arm_control` (`home_service.py`).

## Environment (WSL2)

1. Attach the ST-Link to WSL. Run these in an admin PowerShell on Windows:
   ```
   usbipd list
   usbipd bind --busid <id>
   usbipd attach --wsl --busid <id>
   ```
   The board then shows up as `/dev/ttyACM0`.
2. Run `sudo usermod -aG dialout $USER`, then log in again.
3. Check the link without ROS: in `robot_proto.py`, `Link("/dev/ttyACM0")` must receive STATE frames.

## Testing without hardware

`Software/test_codes/stm_sim/stm_sim.py -v` simulates the driver board on the virtual port `/tmp/ttySTM`. Use `/tmp/ttySTM` as the `port` parameter instead of `/dev/ttyACM0`. See `Software/test_codes/stm_sim/README.md`.

To run the integration tests in `test/test_sim_link.cpp` and `test/test_arm_system.cpp`, start the simulator first in its own terminal. Then run:

```
colcon build --packages-select arm_hardware
colcon test --packages-select arm_hardware && colcon test-result --verbose
```

The simulator keeps its state between test runs. Every test therefore starts by putting it back into a known state. `SimArmSystem` does this through `activate()`.

**Only one program may use the port at a time.** If a launch is still running on `/tmp/ttySTM`, the tests and the launch steal bytes from each other, and you get CRC errors, seq gaps or failed tests. To test in parallel anyway, start a second simulator with `--link /tmp/ttySTM2` and pass `port:=/tmp/ttySTM2`. Use a different `ROS_DOMAIN_ID` for the second launch.

### End-to-end test with the trajectory controller

With the simulator running:

```
colcon build --packages-select arm_hardware && source install/setup.bash
ros2 launch arm_hardware hw_test.launch.py                   # port:=/dev/ttyACM0 for the board
# second terminal:
ros2 control list_hardware_components                        # ArmSystem: active
ros2 topic echo /joint_states --once                         # positions from the STM
ros2 run arm_hardware test_trajectory.py                     # all joints 0 -> 0.3 -> 0 rad, prints PASS
```

In the simulator log (`-v`) you should see `CONFIG ... -> ok`, `SETZERO -> ok`, `ENABLE 1 -> ok` and then RUNNING. The DIAG line should show `rej=0`. Ctrl+C on the launch sends ENABLE 0, and the simulator goes to DISABLED.

The joint parameters in `test/bringup/arm_test.urdf.xacro` are placeholders (gear ratio 20).

### Recovering from FAULT

`read()` only logs a FAULT (JUMP, STARVED, stall, driver error), and `write()` stops sending. To recover, deactivate and activate the component again. `on_activate` then sends CLEARFAULT and keeps the position, without a new SETZERO:

```
ros2 control set_hardware_component_state ArmSystem inactive
ros2 control set_hardware_component_state ArmSystem active
```

## Hooking it into robot_moveit_config (not done yet)

1. **`config/robot_armV2SLDASM.ros2_control.xacro`:**
   - Add a macro parameter `use_mock_hardware`.
   - Use `<xacro:if>` to select `mock_components/GenericSystem` and `<xacro:unless>` to select `arm_hardware/ArmSystem`.
   - Give `ArmSystem` these hardware params:
     ```xml
     <plugin>arm_hardware/ArmSystem</plugin>
     <param name="port">/dev/ttyACM0</param>
     <param name="baudrate">115200</param>
     <param name="period_ms">10</param>
     <param name="set_zero_on_activate">true</param>
     ```
   - Give each joint these params:
     ```xml
     <param name="axis">0</param>              <!-- STM axis 0..5 = TMC1..TMC6 -->
     <param name="gear_ratio">20.0</param>     <!-- placeholder, measure per axis! -->
     <param name="direction">1</param>
     <param name="max_velocity">1.0</param>    <!-- rad/s, see joint_limits.yaml -->
     <param name="max_acceleration">2.0</param><!-- rad/s^2 -->
     ```
2. **`config/robot_armV2SLDASM.urdf.xacro`:**
   - Add `<xacro:arg name="use_mock_hardware" default="true"/>` and pass it into the macro.
   - Rename `FakeSystem` to `ArmSystem`.
3. **New file `launch/real.launch.py`:**
   ```python
   moveit_config = (MoveItConfigsBuilder("robot_armV2SLDASM", package_name="robot_moveit_config")
                    .robot_description(mappings={"use_mock_hardware": "false"})
                    .to_moveit_configs())
   return generate_demo_launch(moveit_config)
   ```
4. **`package.xml`:** add `<exec_depend>arm_hardware</exec_depend>`.
5. **`config/ros2_controllers.yaml`:**
   - Keep `update_rate: 100`. This must match `period_ms = 10`, and `on_init` refuses anything else.
   - Replace `open_loop_control: true` (deprecated in Jazzy) with `interpolate_from_desired_state: true`, as in `test/bringup/controllers.yaml`. The STATE position lags 3–4 periods behind the command.

## What comes next

1. **Milestone 6: MoveIt integration** (section above). Use Plan & Execute in RViz against the simulator, then against the board.
2. **FAULT handling.**
   - `read()` returns ERROR on FAULT, and an `on_error()` sends ENABLE 0. The controller_manager then stops the controllers, instead of the JTC carrying on with commands that go nowhere.
   - Today a FAULT is only logged (see "Recovering from FAULT").
3. **First run on the board.** Start with motors that are not mounted to the mechanics.
   - Measure gear ratio, direction and joint-to-TMC mapping per axis.
   - Take `max_velocity` and `max_acceleration` from `joint_limits.yaml`.
4. **Milestone 7: homing.**
   - Use `start_homing()` instead of SETZERO, with `homing_offsets` from `offsets.yaml`.
   - Add the service in `arm_control/home_service.py`.
   - This needs `Home_Service()` in the firmware.
5. **Later:** software limits on the STM, StallGuard, gripper.

## Open points

- **Gear ratio and direction per axis.** Only the placeholder 20:1 exists so far.
- **Joint-to-TMC mapping.** The assumption is the order in `ros2_controllers.yaml`: `shoulder_joint1` = axis 0 … `gripper_turn_joint` = axis 5.
- **First tests.** Run them with a motor that is not mounted to the mechanics. The STM has no software limits yet.
- **StallGuard.** It only works at 2000 steps/s or more (`min_freq_stall`). With MoveIt's default velocity scaling of 0.1, moves run at about 1000 steps/s, so stall detection is off.
- **Gripper.** Later.

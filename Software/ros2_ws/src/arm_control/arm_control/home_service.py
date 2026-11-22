# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
# See LICENSE-SOFTWARE in the project root for details.
"""
Homing service node (PLANNED, structure only, no code yet).

Offers the service /home_arm (std_srvs/srv/Trigger) so homing can be triggered at
any time, e.g. from the command line or an rqt button:

    ros2 service call /home_arm std_srvs/srv/Trigger

Why a separate node: the serial port belongs to the ros2_control plugin
arm_hardware/ArmSystem. The STM accepts HOMING only in state DISABLED, which is
the "inactive" lifecycle state of the hardware component. So this node does not
talk to the STM itself, it only drives the controller_manager:

    1. deactivate arm_controller
       /controller_manager/switch_controller (controller_manager_msgs/srv/SwitchController)
    2. set the hardware component ArmSystem to "inactive"
       /controller_manager/set_hardware_component_state
       (controller_manager_msgs/srv/SetHardwareComponentState)
       -> ArmSystem::on_deactivate(): ENABLE 0 -> STM DISABLED
    3. set ArmSystem back to "active"
       -> ArmSystem::on_activate(): HOMING instead of SETZERO (start_homing()), ENABLE 1
    4. activate arm_controller again
    5. return success / message in the Trigger response

Same thing by hand (until this node exists):

    ros2 control switch_controllers --deactivate arm_controller
    ros2 control set_hardware_component_state ArmSystem inactive
    ros2 control set_hardware_component_state ArmSystem active
    ros2 control switch_controllers --activate arm_controller

To do when implementing:
- add std_srvs and controller_manager_msgs to package.xml
- register the entry point in setup.py ('home_service = arm_control.home_service:main')
- the service calls must be asynchronous (call_async + callback group / executor),
  otherwise the node blocks itself while waiting for the controller_manager
- homing can take long (firmware Home_Service() is still a placeholder!) -> generous timeouts
"""

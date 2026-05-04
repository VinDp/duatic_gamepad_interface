# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from duatic_gamepad_interface.controllers.base_controller import BaseController

import rclpy
from rclpy.action import ActionClient, get_action_names_and_types

from control_msgs.action import ParallelGripperCommand


class GripperController(BaseController):
    """Handles gripper control."""

    OPEN_POSITION = 0.025
    CLOSE_POSITION = 0.0

    def __init__(self, node, duatic_robots_helper):
        super().__init__(node, duatic_robots_helper)

        self.needed_capabilities = ["manipulation"]
        self.needed_low_level_controllers = ["gripper_action_controller"]
        self.gripper_action_suffix = "gripper_action_controller/gripper_cmd"

        # Dictionary to store action clients: {component_name: client}
        self.gripper_clients = {}

        # Track gripper state per component
        self._gripper_states = {}
        self._last_button_state = False

        self._setup_gripper_clients()

    def _setup_gripper_clients(self):
        """Discover and create action clients for all available grippers with retries."""
        max_retries = 20
        retry_count = 0

        while retry_count < max_retries:
            all_actions = get_action_names_and_types(self.node)

            for action_name, _ in all_actions:
                if action_name.endswith(self.gripper_action_suffix):
                    # Extract component from action name; fall back to "default" for non-namespaced setups.
                    component = self.get_arm_from_topic(action_name) or "default"
                    if component not in self.gripper_clients:
                        self.gripper_clients[component] = ActionClient(
                            self.node, ParallelGripperCommand, action_name
                        )
                        self._gripper_states[component] = False
                        self.node.get_logger().info(
                            f"Gripper action client created for {component} on {action_name}"
                        )

            if self.gripper_clients:
                if retry_count > 5:
                    break

            rclpy.spin_once(self.node, timeout_sec=0.2)
            retry_count += 1

    def send_gripper_command(self, component, position: float):
        """Send a goal to a specific gripper action server."""
        client = self.gripper_clients.get(component) or self.gripper_clients.get("default")
        if not client:
            self.node.get_logger().warn(f"No gripper action client for {component}")
            return

        goal = ParallelGripperCommand.Goal()
        goal.command.position = [position]
        client.send_goal_async(goal)
        self.node.get_logger().info(f"Sent gripper command to {component}: {position}")

    def process_input(self, joy_msg):
        # Safety: Only process gripper if deadman is active
        if not self.node.deadman_active:
            return

        # Determine current focus

        current_hlc = self.node.controller_manager.get_current_controller()
        if not current_hlc:
            return

        focus = current_hlc.get_focus()

        # Toggle gripper state with button 0
        if hasattr(joy_msg, "buttons") and len(joy_msg.buttons) > 0:
            button_pressed = bool(joy_msg.buttons[0])
            if button_pressed and not self._last_button_state:
                # Toggle state under the same key used to register the client
                state_key = focus if focus in self._gripper_states else "default"
                is_open = not self._gripper_states.get(state_key, False)
                self._gripper_states[state_key] = is_open
                position = self.OPEN_POSITION if is_open else self.CLOSE_POSITION
                self.send_gripper_command(focus, position)
            self._last_button_state = button_pressed

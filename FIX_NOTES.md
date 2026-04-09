# Fix Notes

This branch includes a fix for unreliable controller mode switching in `gamepad_interface`.

The important detail is that the gamepad node keeps its own high-level controller state, while ROS 2 control manages the low-level controllers. The node periodically polls the active low-level controller set and maps it to the correct high-level gamepad mode. Before the fix, the internal snapshot of active controllers could become stale, so an external `ros2 control switch_controllers` command was sometimes not reflected in the gamepad state.

The fix makes the polling logic refresh the active low-level controller snapshot consistently and re-evaluate the matching high-level mode whenever the controller set changes. That is why switching to `joint_trajectory_controller` now reliably produces the expected high-level controller index inside `gamepad_interface`.

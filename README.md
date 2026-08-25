# CiA 402 ROS 2 controller

This repository contains two ROS 2 Humble packages:

- `cia402_controller`: a `ros2_control` controller that owns the CiA 402
  Controlword and Modes of operation command interfaces.
- `cia402_interfaces`: the semantic drive-command action and continuous drive
  state messages.

Applications request high-level state changes through an action. The action
callbacks only validate and queue requests; the controller `update()` loop
performs every Controlword transition and checks Statusword feedback.

## Hardware interface contract

For every configured joint, the hardware component must export:

| Direction | ros2_control interface |
| --- | --- |
| Command | `<joint>/control_word` |
| Command | `<joint>/modes_of_operation` |
| State | `<joint>/status_word` |
| State | `<joint>/modes_of_operation_display` |

The numeric ros2_control storage is `double`, but all four values must be
finite, integral, and inside the corresponding CiA 402 type range (`uint16`
or `int8`). The controller refuses activation if its first feedback sample is
invalid.

The trajectory controller remains separate and can own `<joint>/position` at
the same time.

## Configure and start

Add the controller to the controller manager configuration. A complete example
is available in
[`cia402_controller/config/cia402_controller.yaml`](cia402_controller/config/cia402_controller.yaml).

```yaml
controller_manager:
  ros__parameters:
    cia402_controller:
      type: cia402_controller/Cia402Controller

cia402_controller:
  ros__parameters:
    joints: [joint1]
    default_timeout: 5.0
    step_timeout: 1.0
    feedback_rate: 20.0
    default_mode_of_operation: 8
    fallback_command: 6
```

Start it alongside, not instead of, the joint trajectory controller:

```bash
ros2 run controller_manager spawner cia402_controller \
  --controller-manager /controller_manager
```

`default_mode_of_operation` is written before enabling operation. The final
Enable operation step is held until Modes of operation display confirms the
same value. The default `8` is Cyclic synchronous position.

Starting the controller first samples each drive and writes the Controlword
that holds its observed state; activation does not automatically disable an
already enabled drive. `fallback_command` selects the command staged on
cancellation, timeout, transition failure, and controller deactivation: `4`
for Disable operation, `5` for Disable voltage, or the default `6` for Quick
stop. Select this policy from the machine's risk assessment and configured
quick-stop behavior (object `0x605A`).

`feedback_rate` controls both action feedback and `drive_states` publication
and must be between 1 Hz and 1000 Hz. `default_timeout` and `step_timeout` are
accumulated from controller update periods, so ROS clock jumps do not bypass
the safety limits.

## Command drives

The action endpoint is:

```text
/<controller_name>/execute_drive_command
```

An empty `joint_names` array selects every joint configured in this controller.
Otherwise, only the named joints are transitioned. Only one action goal can be
active at a time.

| Command | Value | Completion state |
| --- | ---: | --- |
| Shutdown | 1 | Ready to switch on |
| Switch on | 2 | Switched on |
| Enable operation | 3 | Operation enabled |
| Disable operation | 4 | Switched on, or already not enabled |
| Disable voltage | 5 | Switch on disabled |
| Quick stop | 6 | Quick stop active or switch on disabled |
| Fault reset | 7 | Switch on disabled |

CiA 402 "Shutdown" does not power the controller down; it targets Ready to
switch on.

Enable all configured drives with a five-second timeout:

```bash
ros2 action send_goal \
  /cia402_controller/execute_drive_command \
  cia402_interfaces/action/ExecuteDriveCommand \
  "{joint_names: [], command: 3, timeout: {sec: 5, nanosec: 0}}" \
  --feedback
```

Reset a fault on one drive:

```bash
ros2 action send_goal \
  /cia402_controller/execute_drive_command \
  cia402_interfaces/action/ExecuteDriveCommand \
  "{joint_names: [joint1], command: 7, timeout: {sec: 5, nanosec: 0}}" \
  --feedback
```

Fault reset is emitted as a low-high-low bit-7 pulse. Faults are never reset
automatically. A canceled, timed-out, or failed group goal stages the
configured fallback for every selected drive.

Invalid commands, duplicate or unknown joint names, negative timeouts, and a
second concurrent goal are rejected at the action protocol level. Rejected
goals therefore do not contain a custom action result.

## Observe drive feedback

The controller publishes a coherent snapshot of every configured drive while
active:

```bash
ros2 topic echo \
  /cia402_controller/drive_states \
  cia402_interfaces/msg/DriveStateArray
```

Each entry contains the raw Statusword and mode display, decoded state and
flags, last commanded Controlword and mode, and `feedback_valid`. The
`quick_stop_active` field is true only when the complete Statusword pattern
decodes to Quick stop active; bit 5 alone is insufficient because its inverted
signal is also clear in other states. Quick stop completion accepts both Quick
stop active and Switch on disabled because the terminal state depends on
object `0x605A`.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select cia402_controller
colcon test-result --verbose
```

The tests cover Statusword decoding, semantic transition paths, interface
claiming, activation behavior, mode confirmation, invalid feedback, the
fault-reset pulse, plugin loading, cancellation/fallback behavior, concurrent
goal rejection, and an executor-backed action/topic round trip.

## Integration requirements

- The hardware `read()` method must continue returning successfully while a
  drive is in Fault, otherwise the controller cannot issue Fault reset.
- Every physical drive must have unique PDO mappings. Multiple joint resources
  that alias one Controlword/Statusword location cannot be commanded safely.
- A generic mock hardware component does not emulate CiA 402 state transitions;
  action goals need a drive simulator or real hardware feedback.
- Enable the drives successfully before sending position trajectories.

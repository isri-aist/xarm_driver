# xarm_driver

`xarm_driver` provides a UFACTORY xArm hardware driver plugin for
[Unified Robot Interface](https://github.com/isri-aist/unified_robot_interface).

The driver communicates directly with the robot using the official xArm C++
SDK and implements URI's `robot_interface::RobotDriver` API.

## Current features

- Joint-position feedback
- Joint-velocity feedback
- Estimated joint-torque feedback
- Joint-position control through `servoJ()`
- Joint-velocity control through `speedJ()`
- Command validation and xArm SDK error reporting
- Velocity-command timeout to stop the robot if commands are interrupted

Direct joint-torque control through `tauJ()` is not supported because the
public xArm SDK does not provide a supported joint-torque command. See the
[xArm SDK torque-control discussion](https://github.com/xArm-Developer/xArm-CPLUS-SDK/issues/18).

## Dependencies

- [Unified Robot Interface documentation](https://isri-aist.github.io/unified_robot_interface/)
- [xArm C++ SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK)
- A matching xArm RobotModule, such as
  [mc_xarm](https://github.com/isri-aist/mc_xarm)

## Build and install

Install the driver using the same prefix as URI and the xArm C++ SDK:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH=/path/to/uri/install \
  -DCMAKE_INSTALL_PREFIX=/path/to/uri/install

cmake --build build --parallel
cmake --install build
```

## Usage

See the https://isri-aist.github.io/unified_robot_interface/docGettingStarted.html for instructions on configuring and running a robot driver with URI.

Configuration examples are provided for:

- etc/mc_rtc.yaml
- etc/mc_etc_dual_arms.yaml

The robot names used by URI must match the robot instance names loaded by the controller.
The provided configurations use names such as xarm7 and xarm7_2, but a custom controller may use different names.

## Troubleshooting

### xArm SDK command failure

An error such as the following means that the xArm SDK rejected the command.

```
[RobotDriverxArm] set_servo_angle_j failed with code 9
```

Return-code definitions are available in the [xArm SDK API code documentation](https://github.com/xArm-Developer/xArm-CPLUS-SDK/blob/master/doc/xarm_api_code.md).

### Plugin not found

The driver and URI need to have the same install prefix.

```sh
find /path/to/uri/install \
  -name 'libRobotDriverxArm.so' \
  -print
```

Expected location:

```sh
<prefix>/lib/robot_interface/libRobotDriverxArm.so
```

If the plugin is installed elsewhere, rebuild it with `-DCMAKE_INSTALL_PREFIX=/path/to/uri/install`.

### Note

Joint-velocity commands use a finite timeout (0.05s) so the robot stops if commands are no longer received.

## License

See the repository license for details.
The xArm C++ SDK is distributed separately under its own license.

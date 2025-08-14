# tianbot_core ROS2

ROS2 Package for the embedded controller of mobile robots.

## Dependencies

- ROS2 Humble

## Build
Clone and Build the project using the following steps.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/tianbot/tianbot_core.git -b ros2
cd tianbot_core && ./build.sh
```

## Usage

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch tianbot_core tianbot_core.launch.py
```

## Param Set

```bash
export ROBOT_NAME=tianbot     # essential, the namespace of service
python3 scripts/param_set.py param/tom06s 
```

## Debug Command

- input
```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /tianbot/debug_cmd_srv tianbot_core/srv/DebugCmd "{cmd: param get}"
```
- output
```bash
requester: making request: tianbot_core.srv.DebugCmd_Request(cmd='param get')

response:
tianbot_core.srv.DebugCmd_Response(result='az_offset 0.000\r\npid_p 100.000\r\npid_i 10.000\r\npid_d 3.000\r\nmax_speed 5.000\r\npwm_dead_zone  40\r\nrc_dead_zone  30\r\nservo_dir:   1\r\nencoder_dir:   1\r\nticks_per_lap: 4096\r\nmotor_reduction: 7.600\r\n')
```

- input

```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /tianbot/debug_cmd_srv tianbot_core/srv/DebugCmd "{cmd: help}"
```

- output
```bash
requester: making request: tianbot_core.srv.DebugCmd_Request(cmd='help')

response:
tianbot_core.srv.DebugCmd_Response(result='set_az_offset: set accel z-axis offset.\r\nset_servo_dir: set servo direction.\r\nset_enc_dir: set encoder direction.\r\nset_max_speed: set max speed.\r\nset_p: set pid p.\r\nset_i: set pid i.\r\nset_d: set pid d.\r\nset_pwm_dead_zone: set pid d.\r\nset_odom: set odom x y yaw.\r\nset_tpl: set ticks per lap.\r\nreset: reset board.\r\nparam: get param.\r\nset_motor_reduction: set motor reduction.\r\nhelp: list all command.\r\n')
```

| cmd   | example | description |
| :--- | :---  | :---  |
| set_az_offset   | "{cmd: set_az_offset 1.25  }" | set z-axis acceleration offset.|
| set_max_speed   | "{cmd: set_max_speed 10.0}" |  set max speed to 10.0 |
| set_odom        | "{cmd: set_odom 0 0 0 }"    |  set odom x, y, yaw.   |
| set_p           | "{cmd: set_p 0}"  | set pid velocity loop k_p to 0   |
| set_i           | "{cmd: set_i 0}"  | set pid velocity loop k_i to 0   |
| set_d           | "{cmd: set_d 0}"  | set pid velocity loop k_d to 0   |
| reset           | "{cmd: reset}"    | reset board.                     |
| help            | "{cmd: help}"     |                                  |


- input

```bash
ros2 service call  /tianracer/debug_cmd_srv tianbot_core/srv/DebugCmd "{cmd: set_az_offset 1.25}"
```

- output

```bash
waiting for service to become available...
requester: making request: tianbot_core.srv.DebugCmd_Request(cmd='set_az_offset 1.25')

response:
tianbot_core.srv.DebugCmd_Response(result='accel z-axis offset set to 1.250000\r\n')
```

## Project Architecture

### Core Modules
1. **Chassis Control Module**
   - Supports multiple chassis types (differential, omnidirectional, Ackermann, Rover).
   - Related files: `src/chassis.cpp`, `src/differential.cpp`, `src/omni.cpp`, `src/ackermann.cpp`, `src/rover.cpp`.

2. **Communication Module**
   - Provides serial and UDP communication interfaces.
   - Related files: `src/serial.cpp`, `src/udp.cpp`, `src/protocol.cpp`.

3. **Core Logic Module**
   - Main control logic and message handling.
   - Related files: `src/core.cpp`, `src/main.cpp`.

4. **Debug Service Module**
   - Provides debugging command interfaces.
   - Related file: `srv/DebugCmd.srv`.

### Dependencies
- **ROS 2 Core Dependencies**: `rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`.
- **Communication Dependencies**: `tf2`, `tf2_ros`, `tf2_geometry_msgs`.
- **Chassis Control Dependencies**: `ackermann_msgs`.

### Project Structure
```
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── tianbot_core/
│       ├── chassis.h
│       ├── differential.h
│       ├── omni.h
│       ├── ackermann.h
│       ├── rover.h
│       ├── serial.h
│       ├── udp.h
│       ├── protocol.h
│       └── core.h
├── src/
│   ├── chassis.cpp
│   ├── differential.cpp
│   ├── omni.cpp
│   ├── ackermann.cpp
│   ├── rover.cpp
│   ├── serial.cpp
│   ├── udp.cpp
│   ├── protocol.cpp
│   ├── core.cpp
│   └── main.cpp
├── srv/
│   └── DebugCmd.srv
├── launch/
├── param/
└── scripts/
```
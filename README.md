<div align="center">

# sangbay_driver

**ROS Noetic driver for the Sangbay S-M4T gas sensor (H2S and others) over RS485 Modbus RTU —
concentration, alarm state and diagnostics, and it reconnects after an unplug.**

[![CI](https://github.com/DarmawanRobotics/sangbay_driver/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/DarmawanRobotics/sangbay_driver/actions/workflows/ci.yml)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)](https://wiki.ros.org/noetic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/20.04/)
![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)

</div>

---

The sensor sits on a USB-RS485 adapter (9600 8N1, factory address `0xFF`). The Modbus client has
no ROS dependency and also works on its own (`sangbay_read`); the node only uses standard
messages.

## Quick start

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/DarmawanRobotics/sangbay_driver.git
cd ~/catkin_ws
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src --ignore-src -y --rosdistro noetic
catkin_make && source devel/setup.bash

sudo usermod -aG dialout $USER                       # serial access, then log in again
roslaunch sangbay_driver sangbay.launch port:=/dev/ttyUSB0
```

Use `/dev/serial/by-id/usb-...` as `port` so the name does not change between boots.

## Interface

Everything under `/sangbay` (`ns:=`).

| Topic | Type | |
|---|---|---|
| `concentration` | `std_msgs/Float32` | ppm, every reading (`rate`, 1 Hz) |
| `status` | `std_msgs/String` | `NORMAL`, `PREHEATING (12.3 s)` or the alarms, e.g. `STEL, HIGH` |
| `alarm` | `std_msgs/Bool` | any alarm flag (latched) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | gas, raw value, decimals, status word, every flag; WARN on alarm or preheating, ERROR on sensor error or no answer |

Alarm flags: `low`, `high`, `stel`, `twa`, `over_range`, `sensor_error`, `high_protection`.

## Usage

```bash
rostopic echo /sangbay/concentration
rostopic echo /sangbay/status
rosrun rqt_runtime_monitor rqt_runtime_monitor        # diagnostics

# without ROS: check the wiring, -v prints the Modbus frames
rosrun sangbay_driver sangbay_read --port /dev/ttyUSB0 -v
```

```python
import rospy
from std_msgs.msg import Bool, Float32

rospy.init_node('h2s_monitor')
rospy.Subscriber('/sangbay/concentration', Float32, lambda m: rospy.loginfo('%.2f ppm', m.data))
rospy.Subscriber('/sangbay/alarm', Bool, lambda m: m.data and rospy.logwarn('H2S alarm'))
rospy.spin()
```

Parameters are commented in [sangbay.yaml](config/sangbay.yaml): `port`, `baud`, `address`,
`rate`, `timeout`, `retries`, `reconnect_sec`, `max_failures`, `frame_id`, `expected_gas`.

## Registers

| Register | |
|---|---|
| `0x2027` | gas code (`0x02` = H2S) |
| `0x2031` | decimal places: ppm = raw / 10^decimals |
| `0x6000` | status: bit 15 = preheating done (else bits 0-14 = remaining 0.1 s), bits 0-6 = low, high, STEL, TWA, over range, sensor error, high protection |
| `0x6001` | concentration, raw |

## Troubleshooting

| Symptom | Fix |
|---|---|
| `connect to /dev/ttyUSB0: ... Permission denied` | `dialout` group (above) |
| `read: no answer` | A/B swapped, sensor not powered, wrong `address` or `baud`; try `sangbay_read -v` |
| `CRC error` / `short answer` | noise on the bus: shorter cable, termination, lower `rate` |
| `sensor reports gas 0x.., expected H2S` | another sensor model; set `expected_gas` (or empty) |

## Develop

```bash
catkin_make run_tests && catkin_test_results build/test_results
python3 src/sangbay_driver/test/fake_sensor.py --link /tmp/sangbay_fake &    # simulated sensor
roslaunch sangbay_driver sangbay.launch port:=/tmp/sangbay_fake address:=1
```

Unit tests cover the Modbus frames, the status decoding and the client against the simulated
sensor (also over a pseudo terminal); the rostest runs the launch file against it. CI runs both
on `ros:noetic`.

Released under the [Apache License 2.0](LICENSE).

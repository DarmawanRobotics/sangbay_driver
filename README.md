<div align="center">

# sangbay_driver

**ROS Noetic driver for Sangbay gas sensors — S-M4, S-M4T and K-5S Modbus modules (any gas,
several per RS485 bus) and the GT-TS01A / SLTD-CH4-100 laser methane sensors, all from one
config file.**

[![CI](https://github.com/DarmawanRobotics/sangbay_driver/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/DarmawanRobotics/sangbay_driver/actions/workflows/ci.yml)
[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)](https://wiki.ros.org/noetic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/20.04/)
![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)

</div>

---

List your sensors in a YAML file and every one gets its own topics. Sensors on the same
serial port share it (one thread per port), a sensor that stops answering only affects itself,
and unplugged adapters are reopened. The protocol code has no ROS dependency and is also
usable from `sangbay_tool`. Only standard messages are used.

**Docs:** [supported sensors and protocols](docs/sensors.md) · [first run](docs/first_run.md) ·
[datasheets](docs/datasheets)

| `type` | Products | Link | Unit |
|---|---|---|---|
| `module` | S-M4, S-M4T (TTL), S-M4T 485, K-5S, with any head (S4-H2S-100, MJC4-2.8J, N-CH4, ...) | Modbus RTU 9600, address 0..254 | from the module (ppm, %LEL, %VOL, ...) |
| `gt_ts01a` | GT-TS01A laser methane | ASCII 115200, every 2 s | %VOL |
| `sltd_ch4` | SLTD-CH4-100 laser methane remote sensor (100 m) | binary 115200, every 50 ms | ppm·m |

## Quick start

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/DarmawanRobotics/sangbay_driver.git
cd ~/catkin_ws
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src --ignore-src -y --rosdistro noetic
catkin_make && source devel/setup.bash
sudo usermod -aG dialout $USER                       # serial access, then log in again

# one module at the factory address
roslaunch sangbay_driver sangbay.launch port:=/dev/ttyUSB0
# your own list
roslaunch sangbay_driver sangbay.launch config:=/path/to/sensors.yaml
```

## Several sensors

```yaml
rate: 1.0
sensors:
  - {name: h2s, port: /dev/ttyUSB0, address: 1, expected_gas: H2S, alarm_threshold: 10}
  - {name: co,  port: /dev/ttyUSB0, address: 2, expected_gas: CO}
  - {name: o2,  port: /dev/ttyUSB0, address: 3, expected_gas: O2}
  - {name: ch4_laser,  type: gt_ts01a, port: /dev/ttyUSB1}
  - {name: ch4_remote, type: sltd_ch4, port: /dev/ttyUSB2, mode: dynamic, pointer: true}
```

Every module leaves the factory on address 255. To put several on one RS485 bus, connect them
**one at a time** and give each its own address:

```bash
rosrun sangbay_driver sangbay_tool set-address --port /dev/ttyUSB0 --address 255 --new 1
rosrun sangbay_driver sangbay_tool scan --port /dev/ttyUSB0      # afterwards, all on the bus
```

All keys are explained in [example_multi.yaml](config/example_multi.yaml). Use
`/dev/serial/by-id/usb-...` as `port` so the names stay the same between boots.

## Interface

Per sensor, under `/sangbay/<name>/`:

| Topic | Type | |
|---|---|---|
| `concentration` | `std_msgs/Float32` | in `unit`; not published while the module preheats |
| `unit` | `std_msgs/String` | `ppm`, `%LEL`, `%VOL`, `ppm*m`, ... (latched) |
| `status` | `std_msgs/String` | `NORMAL`, `PREHEATING (12.3 s)`, `HIGH, STEL`, `SENSOR-ERROR`, `NO DATA: ...` (latched) |
| `alarm` | `std_msgs/Bool` | gas alarm: the module's alarm bits, over range, or `alarm_threshold` (latched) |
| `fault` | `std_msgs/Bool` | sensor damaged, laser abnormal, or no data (latched) |
| `temperature`, `pressure` | `std_msgs/Float32` | GT-TS01A: compensation temperature °C, kPa |
| `light_intensity`, `laser_temperature` | `std_msgs/Float32` | SLTD-CH4-100: return signal, raw laser temperature |

`/diagnostics` has one entry per sensor (`sangbay: <name>`, hardware id `port#address`) with
every detail: gas, raw value, decimals, status word, each flag. OK, WARN on alarm or preheating,
ERROR on fault or no data.

## Usage

```bash
rostopic echo /sangbay/h2s/concentration
rostopic echo /sangbay/h2s/status
rosrun rqt_runtime_monitor rqt_runtime_monitor

# without ROS
rosrun sangbay_driver sangbay_tool scan --port /dev/ttyUSB0
rosrun sangbay_driver sangbay_tool read --port /dev/ttyUSB0 --address 1 -v   # -v: Modbus frames
rosrun sangbay_driver sangbay_tool read --type gt_ts01a --port /dev/ttyUSB1
rosrun sangbay_driver sangbay_tool read --type sltd_ch4 --port /dev/ttyUSB2
```

```python
import rospy
from std_msgs.msg import Bool, Float32

rospy.init_node('gas_monitor')
for name in ('h2s', 'co'):
    rospy.Subscriber('/sangbay/%s/concentration' % name, Float32,
                     lambda m, n=name: rospy.loginfo('%s %.2f', n, m.data))
    rospy.Subscriber('/sangbay/%s/alarm' % name, Bool,
                     lambda m, n=name: m.data and rospy.logwarn('%s alarm', n))
rospy.spin()
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `cannot open /dev/ttyUSB0: ... Permission denied` | `dialout` group (above) |
| `address 1: no answer` | A/B swapped, no power, wrong address: `sangbay_tool scan` |
| several modules answer garbage | two modules on the same address: set them one at a time |
| `CRC error` / `short answer` | noise: shorter cable, 120 Ω termination at both ends, lower `rate` |
| `module reports CO, expected H2S` | a different head or module on that address |
| GT-TS01A / SLTD `no data` | 115200 baud, TX/RX crossed, 5 V supply; see [first_run.md](docs/first_run.md) |

## Develop

```bash
catkin_make run_tests && catkin_test_results build/test_results
rosrun sangbay_driver fake_devices.py --bus /tmp/sangbay_bus --gt /tmp/sangbay_gt --sltd /tmp/sangbay_sltd &
roslaunch sangbay_driver sangbay.launch config:=$(rospack find sangbay_driver)/config/example_fake.yaml
```

The unit tests check the frames against the datasheet examples and run the buses against
simulated modules (several on one line), a GT-TS01A and an SLTD-CH4-100 on pseudo terminals,
including dropouts and reconnects; the rostest runs the launch file against them. CI runs both
on `ros:noetic`.

Released under the [Apache License 2.0](LICENSE).

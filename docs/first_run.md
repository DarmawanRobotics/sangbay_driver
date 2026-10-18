# First run on the hardware

The driver follows the datasheets and is tested against simulated devices
([fake_devices.py](../test/fake_devices.py)). Some points the datasheets leave open; check
them once and note what you see.

## Modules

```bash
rosrun sangbay_driver sangbay_tool scan --port /dev/ttyUSB0
rosrun sangbay_driver sangbay_tool read --port /dev/ttyUSB0 --address <a> -v
```

| # | Check | Expected |
|---|---|---|
| 1 | `scan` with one factory module | found on 255 (and maybe on every address: then 255 answers for all) |
| 2 | `info` | gas and unit match the label; decimals plausible |
| 3 | `read` right after power on | `PREHEATING (... s)` counting down, then `NORMAL` |
| 4 | `set-address --new 1`, then `scan` | only address 1 |
| 5 | two modules with their own addresses on one bus, `scan` | both, nothing else |
| 6 | test gas or the module's alarm | `alarm` and `status` change in ROS |

If a module's concentration never leaves 0 or jumps between huge values, note the raw value
(`-v`): the register might be signed for that gas.

## GT-TS01A

```bash
rosrun sangbay_driver sangbay_tool read --type gt_ts01a --port /dev/ttyUSB1
```

A line every 2 s with `NORMAL`. `no data (last: not a GT-TS01A line ...)`: the format differs;
send the printed line along with an issue.

## SLTD-CH4-100

```bash
rosrun sangbay_driver sangbay_tool read --type sltd_ch4 --port /dev/ttyUSB2
```

| What you see | Meaning |
|---|---|
| values 20 times a second | all fine |
| `no data (last: CRC error: CC 09 ...)` | the checksum is computed differently: set `check_crc: false` for the sensor and report the frame |
| `no data` with nothing after | measurement did not start: check TX/RX and 5 V; the green laser should light up |

`light_intensity` shows how much laser light comes back: aim at a surface and note the values
for good and bad targets. `laser_temperature` is the raw value (units not given in the
specification).

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sangbay_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0
-----
* Any number of sensors from one config: S-M4, S-M4T (TTL / 485), K-5S modules (several per
  RS485 bus), GT-TS01A and SLTD-CH4-100 laser methane sensors.
* Topics per sensor: concentration, unit, status, alarm, fault (+ laser extras); diagnostics.
* Unit and full gas table from the module; alarm thresholds; no concentration while preheating.
* ``sangbay_tool``: scan, info, read, set-address (replaces ``sangbay_read``).
* Simulated devices, unit tests, rostest.

0.1.0
-----
* S-M4T H2S reader as a ROS node.

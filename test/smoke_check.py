#!/usr/bin/env python3
"""rostest: the node with config/example_fake.yaml against fake_devices.py."""

import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool, Float32, String


def get(topic, kind, timeout=20):
    return rospy.wait_for_message('/sangbay/' + topic, kind, timeout=timeout)


class TestSmoke(unittest.TestCase):
    def test_modules(self):
        self.assertAlmostEqual(get('h2s/concentration', Float32).data, 12.5, places=3)
        self.assertEqual(get('h2s/unit', String).data, 'ppm')
        self.assertEqual(get('h2s/status', String).data, 'NORMAL')
        self.assertFalse(get('h2s/alarm', Bool).data)
        self.assertFalse(get('h2s/fault', Bool).data)
        self.assertAlmostEqual(get('co/concentration', Float32).data, 3.0)
        self.assertTrue(get('co/alarm', Bool).data)        # alarm_threshold 2
        self.assertEqual(get('co/status', String).data, 'THRESHOLD')
        self.assertEqual(get('ch4/unit', String).data, '%LEL')

    def test_lasers(self):
        self.assertAlmostEqual(get('ch4_laser/concentration', Float32).data, 0.12, places=4)
        self.assertEqual(get('ch4_laser/unit', String).data, '%VOL')
        self.assertAlmostEqual(get('ch4_laser/pressure', Float32).data, 101.3, places=3)
        self.assertEqual(get('ch4_remote/concentration', Float32).data, 250.0)
        self.assertEqual(get('ch4_remote/unit', String).data, 'ppm*m')
        self.assertEqual(get('ch4_remote/light_intensity', Float32).data, 3000.0)

    def test_diagnostics(self):
        seen = {}
        end = rospy.Time.now() + rospy.Duration(30)
        while rospy.Time.now() < end and len(seen) < 5:
            for st in rospy.wait_for_message('/diagnostics', DiagnosticArray, timeout=10).status:
                # 'gas' is only there once the sensor has a reading
                if st.name.startswith('sangbay: ') and any(kv.key == 'gas' for kv in st.values):
                    seen[st.name[9:]] = st
        self.assertEqual(sorted(seen), ['ch4', 'ch4_laser', 'ch4_remote', 'co', 'h2s'])
        self.assertEqual(seen['h2s'].hardware_id, '/tmp/sangbay_bus#1')
        self.assertEqual(seen['h2s'].level, 0)
        self.assertEqual(seen['co'].level, 1)  # alarm: WARN


if __name__ == '__main__':
    rospy.init_node('smoke_check')
    rostest.rosrun('sangbay_driver', 'smoke', TestSmoke)

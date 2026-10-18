#!/usr/bin/env python3
"""rostest: the node against fake_sensor.py."""

import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool, Float32, String


class TestSmoke(unittest.TestCase):
    def test_topics(self):
        conc = rospy.wait_for_message('/sangbay/concentration', Float32, timeout=20)
        self.assertAlmostEqual(conc.data, 12.5, places=3)
        status = rospy.wait_for_message('/sangbay/status', String, timeout=5)
        self.assertEqual(status.data, 'NORMAL')
        alarm = rospy.wait_for_message('/sangbay/alarm', Bool, timeout=5)
        self.assertFalse(alarm.data)

        def has_values(msg):
            return msg.status and any(kv.key == 'gas' for kv in msg.status[0].values)

        deadline = rospy.Time.now() + rospy.Duration(10)
        diag = None
        while rospy.Time.now() < deadline:
            diag = rospy.wait_for_message('/diagnostics', DiagnosticArray, timeout=5)
            if has_values(diag):
                break
        values = dict((kv.key, kv.value) for kv in diag.status[0].values)
        self.assertEqual(values['gas'], 'H2S')
        self.assertEqual(values['raw'], '125')


if __name__ == '__main__':
    rospy.init_node('test_smoke')
    rostest.rosrun('sangbay_driver', 'smoke', TestSmoke)

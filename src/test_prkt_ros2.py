#!/usr/bin/env python

'''
It might make sense to just do testing in the same folder
'''
import rospy

import sys
sys.path.append('/home/buck/ros_workspace/src')

import unittest

from prkt_core_v2 import FastSLAM
from prkt_ros import CamSlam360

class RosFunctionalityTest(unittest.TestCase):
    def test(self):
        self.assertEqual(1, 1)

    def test_initialization(self):
        cs = CamSlam360()
        self.assertTrue(isinstance(cs.core, FastSLAM))

    def test_slam_exists(self):
        cs = CamSlam360()
        self.assertTrue(isinstance(cs.core, FastSLAM))
        self.assertTrue(isinstance(cs.cam_sub, rospy.Subscriber))
        self.assertTrue(isinstance(cs.twist_sub, rospy.Subscriber))

if __name__ == '__main__':
    # rospy.init_node('test_node_010a0as12asdjkfobu')
    # rospy.loginfo('sys.version')
    # import sys
    # rospy.loginfo(sys.version.split(' ')[0])

    import rostest
    rostest.rosrun('crispy_parakeet', 'test_ooos', RosFunctionalityTest)
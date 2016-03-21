#!/usr/bin/env python
import sys
sys.path.append('/home/buck/ros_workspace/src')

import unittest

from crispy_parakeet.src.prkt_core_v2 import FastSLAM
from prkt_ros import CamSlam360

class MyTest(unittest.TestCase):
    def test(self):
        print('sys.version')
        import sys
        print(sys.version.split(' ')[0])
        self.assertEqual(1, 1)

    # def test_initialization(self):
    #     cs = CamSlam360()
    #     self.assertTrue(isinstance(cs.core, FastSLAM))

if __name__ == '__main__':
    import rostest
    rostest.rosrun('crispy_parakeet', 'test_ooos', MyTest)
#!/usr/bin/env python

'''
It might make sense to just do testing in the same folder
'''
import rospy

import sys
sys.path.append('/home/buck/ros_workspace/src')

import unittest
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from prkt_core_v2 import FastSLAM, FilterParticle, Feature
from prkt_ros import CamSlam360
from viz_feature_sim.msg import Blob

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

class prktFastSLAMTest(unittest.TestCase):
    # it will be very hard to test the methods in the FastSLAM class alone
    #   because they don't return any values and/or they involve random noise
    def test_initilization(self):
        fs = FastSLAM()
        self.assertTrue(isinstance(fs.last_control, Twist))
        self.assertTrue(isinstance(fs.last_update, rospy.Time))
        self.assertTrue(isinstance(fs.particles, list))
        self.assertTrue(isinstance(fs.Qt, np.ndarray))

class prktFilterParticleTest(unittest.TestCase):
    def test_initialization(self):
        particle = FilterParticle()
        self.assertTrue(isinstance(particle.state, Odometry))
        self.assertTrue(isinstance(particle.feature_set, dict))
        self.assertTrue(isinstance(particle.potential_features, dict))
        self.assertTrue(isinstance(particle.hypothesis_set, dict))
        self.assertEqual(particle.weight, 1)
        self.assertEqual(particle.next_id, 1)

    def test_get_feature_by_id(self):
        particle = FilterParticle()
        f0 = Feature()
        f0.arbitrary_id = 'tangled'
        particle.feature_set[2] = f0

        is_f0 = particle.get_feature_by_id(2)
        self.assertEqual(f0.arbitrary_id, is_f0.arbitrary_id)

        f1 = Feature()
        f1.arbitrary_id = 'snow white'
        particle.potential_features[-3] = f1

        is_f1 = particle.get_feature_by_id(-3)
        self.assertEqual(f1.arbitrary_id, is_f1.arbitrary_id)

    def test_probability_of_match(self):
        particle = FilterParticle()

        # two 0 cases: colors far apart, bearing far off
        state = Odometry()
        blob_color = Blob()
        blob_color.color.r = 0
        blob_color.color.g = 0
        blob_color.color.b = 255
        feature = Feature(mean=np.array([1,0,255,0,0]))

        result_color = particle.probability_of_match(state, blob_color, feature)

        self.assertEqual(result_color, 0.0)

        blob_bearing = Blob()
        blob_bearing.bearing = math.pi
        feature = Feature(mean=np.array([1,0,0,0,0]))

        result_bearing = particle.probability_of_match(state, blob_bearing, feature)

        self.assertEqual(result_bearing, 0.0)

class prktFeatureTest(unittest.TestCase):
    def test_initialization(self):
        feature = Feature()
        self.assertTrue(isinstance(feature.mean, np.ndarray))
        self.assertTrue(isinstance(feature.covar, np.ndarray))
        self.assertTrue(isinstance(feature.identity, np.ndarray))
        self.assertEqual(feature.update_count, 0)

if __name__ == '__main__':
    # rospy.init_node('test_node_010a0as12asdjkfobu')
    # rospy.loginfo('sys.version')
    # import sys
    # rospy.loginfo(sys.version.split(' ')[0])

    import rostest
    rostest.rosrun('crispy_parakeet', 'test_prkt_ros_functionality', RosFunctionalityTest)
    rostest.rosrun('crispy_parakeet', 'test_prkt_FastSLAM', prktFastSLAMTest)
    rostest.rosrun('crispy_parakeet', 'test_prkt_Feature', prktFeatureTest)
    rostest.rosrun('crispy_parakeet', 'test_prkt_FilterParticle', prktFilterParticleTest)
    
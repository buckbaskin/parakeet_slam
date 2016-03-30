#!/usr/bin/env python

'''
It might make sense to just do testing in the same folder
'''
import rospy

import sys
sys.path.append('/home/buck/ros_workspace/src')

import math
import numpy as np
import unittest

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from prkt_core_v2 import FastSLAM, FilterParticle, Feature
from prkt_ros import CamSlam360
from utils import heading_to_quaternion
from viz_feature_sim.msg import Blob

class RosFunctionalityTest(unittest.TestCase):
    def test(self):
        self.assertEqual(1, 1)

    def test_initialization(self):
        cs = CamSlam360()
        self.assertIsInstance(cs.core, FastSLAM)

    def test_slam_exists(self):
        cs = CamSlam360()
        self.assertIsInstance(cs.core, FastSLAM)
        self.assertIsInstance(cs.cam_sub, rospy.Subscriber)
        self.assertIsInstance(cs.twist_sub, rospy.Subscriber)

class prktFastSLAMTest(unittest.TestCase):
    # it will be very hard to test the methods in the FastSLAM class alone
    #   because they don't return any values and/or they involve random noise
    def test_initilization(self):
        fs = FastSLAM()
        self.assertIsInstance(fs.last_control, Twist)
        self.assertIsInstance(fs.last_update, rospy.Time)
        self.assertIsInstance(fs.particles, list)
        self.assertIsInstance(fs.Qt, np.ndarray)

class prktFilterParticleTest(unittest.TestCase):
    def test_initialization(self):
        particle = FilterParticle()
        self.assertIsInstance(particle.state, Odometry)
        self.assertIsInstance(particle.feature_set, dict)
        self.assertIsInstance(particle.potential_features, dict)
        self.assertIsInstance(particle.hypothesis_set, dict)
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

    def test_probability_of_match_color(self):
        particle = FilterParticle()
        state = Odometry()
        # two 0 cases: colors far apart, _bearing_ far off
        blob_color = Blob()
        blob_color.color.r = 255
        feature = Feature(mean=np.array([1,0,0,0,0]))

        self.assertIsInstance(feature.mean, np.ndarray)

        result_color = particle.probability_of_match(state, blob_color, feature)

        self.assertEqual(result_color, 0.0)

    def test_probability_of_match_bearing(self):
        particle = FilterParticle()
        state = Odometry()
        # two 0 cases: colors far apart, _bearing_ far off
        blob_bearing = Blob()
        blob_bearing.bearing = math.pi
        feature = Feature(mean=np.array([1,0,0,0,0]))

        self.assertIsInstance(feature.mean, np.ndarray)

        result_bearing = particle.probability_of_match(state, blob_bearing, feature)

        self.assertEqual(result_bearing, 0.0)

    def test_prob_position_match(self):
        particle = FilterParticle()

        f_mean = np.array([1,0,0,0,0])
        f_covar = np.array([[.1,0],
                            [0,.1]])
        s_x = 0.0
        s_y = 0.0
        bearing = 0.0

        line_up_result1 = particle.prob_position_match(f_mean, f_covar, s_x, s_y, bearing)
        self.assertTrue(line_up_result1 > 1.59)

        bearing = 1.0
        line_up_result2 = particle.prob_position_match(f_mean, f_covar, s_x, s_y, bearing)
        self.assertTrue(line_up_result2 < 0.05)
        self.assertTrue(line_up_result1 > line_up_result2)

        bearing = -1.0
        line_up_result2 = particle.prob_position_match(f_mean, f_covar, s_x, s_y, bearing)
        self.assertTrue(line_up_result2 < 0.05)
        self.assertTrue(line_up_result2 > 0.0)
        self.assertTrue(line_up_result1 > line_up_result2)

        bearing = math.pi
        line_up_result3 = particle.prob_position_match(f_mean, f_covar, s_x, s_y, bearing)
        self.assertTrue(line_up_result2 > line_up_result3)

    def test_closest_point(self):
        print('test_closest_point')
        particle = FilterParticle()
        f_x = 1.0
        f_y = 0.0
        s_x = 0.0
        s_y = 0.0
        bearing = 0.0

        c_x, c_y = particle.closest_point(f_x, f_y, s_x, s_y, bearing)
        self.assertEqual(c_x, 1.0)
        self.assertEqual(c_y, 0.0)

        bearing = math.pi/2

        c_x, c_y = particle.closest_point(f_x, f_y, s_x, s_y, bearing)
        self.assertTrue(c_x < .00001)
        self.assertTrue(c_y < .00001)

        bearing = math.pi

        c_x, c_y = particle.closest_point(f_x, f_y, s_x, s_y, bearing)
        self.assertEqual(c_x , 0.0)
        self.assertEqual(c_y , 0.0)

        bearing = math.pi*3.0/4.0

        c_x, c_y = particle.closest_point(f_x, f_y, s_x, s_y, bearing)
        self.assertEqual(c_x , 0.0)
        self.assertEqual(c_y , 0.0)

        bearing = -math.pi*3.0/4.0

        c_x, c_y = particle.closest_point(f_x, f_y, s_x, s_y, bearing)
        self.assertEqual(c_x , 0.0)
        self.assertEqual(c_y , 0.0)

    def test_prob_color_match(self):
        # Note to self: check and see if the 0 is a problem for high covariance
        particle = FilterParticle()
        f_mean = np.array([0,0,255,0,0])
        f_covar = list([[0,0,0,0,0],
                        [0,0,0,0,0],
                        [0,0,5,0,0],
                        [0,0,0,5,0],
                        [0,0,0,0,5]])
        f_covar = np.array(f_covar)
        blob = Blob()
        blob.color.r = 255
        blob.color.g = 0
        blob.color.b = 0

        result1 = particle.prob_color_match(f_mean, f_covar, blob)
        self.assertTrue(result1 > 0.005)

        blob.color.r = 250

        result2 = particle.prob_color_match(f_mean, f_covar, blob)
        self.assertTrue(result2 < result1)

        blob.color.b = 5

        result3 = particle.prob_color_match(f_mean, f_covar, blob)
        self.assertTrue(result3 < result2)

        blob.color.r = 200

        result4 = particle.prob_color_match(f_mean, f_covar, blob)
        self.assertTrue(result4 < result2)

    def test_add_hypothesis(self):
        # TODO(buckbaskin): this is based on other code that needs tested first
        pass

    def test_find_nearest_reading(self):
        particle = FilterParticle()
        state1 = Odometry() # 0,0
        blob1 = Blob()
        blob1.bearing = .1
        
        state2 = Odometry() # 0,1
        state2.pose.pose.position.y = 1
        blob2 = Blob()
        blob2.bearing = -.1

        particle.potential_features[-1] = (state1, blob1)

        min_dist_id = particle.find_nearest_reading(state2, blob2)
        self.assertEqual(min_dist_id, -1)

        # parallel to state2 option, should not match
        state3 = Odometry()
        state3.pose.pose.position.y = 1
        blob3 = Blob()
        blob3.bearing = -.1
        particle.potential_features[-3] = (state3, blob3)

        min_dist_id = particle.find_nearest_reading(state2, blob2)
        self.assertEqual(min_dist_id, -1)

        # wrong color option, should not match
        state4 = Odometry() # 0,0
        blob4 = Blob()
        blob4.bearing = .1
        blob4.color.r = 255
        blob4.color.g = 255
        blob4.color.b = 255
        particle.potential_features[-4] = (state4, blob4)

        min_dist_id = particle.find_nearest_reading(state2, blob2)
        self.assertEqual(min_dist_id, -1)

        # doesn't intersect state 2, should not match
        state5 = Odometry() # 0,0
        state5.pose.pose.position.y = 100
        blob5 = Blob()
        blob5.bearing = -.1
        particle.potential_features[-5] = (state4, blob4)

        min_dist_id = particle.find_nearest_reading(state2, blob2)
        self.assertEqual(min_dist_id, -1)

    def test_reading_distance_function(self):
        particle = FilterParticle()
        state1 = Odometry()
        blob1 = Blob()

        # lines are parallel, should have no intersection
        state2 = Odometry()
        state2.pose.pose.position.y = 1
        blob2 = Blob()
        result2 = particle.reading_distance_function(state1, blob1, state2, blob2)
        self.assertEqual(result2, float('inf'))

        # lines don't intersect should have no intersection
        state3 = Odometry()
        state3.pose.pose.position.y = 1
        blob3 = Blob()
        blob3.bearing = 1.0
        result3 = particle.reading_distance_function(state1, blob1, state3, blob3)
        self.assertEqual(result3, float('inf'))

        # lines intersect, color is the same, distance is 0
        state4 = Odometry()
        state4.pose.pose.position.y = 1
        blob4 = Blob()
        blob4.bearing = -1.0
        result4 = particle.reading_distance_function(state1, blob1, state4, blob4)
        self.assertEqual(result4, 0.0)

        # lines intersect, color is close, distance is small
        state5 = Odometry()
        state5.pose.pose.position.y = 1
        blob5 = Blob()
        blob5.bearing = -1.0
        blob5.color.r = 5
        result5 = particle.reading_distance_function(state1, blob1, state5, blob5)
        self.assertTrue(result5 > 0.0)

    def test_ray_intersect(self):
        particle = FilterParticle()
        x1 = 0.0
        y1 = 0.0
        b1 = 0.0

        x2 = 1.0
        y2 = 0.0
        b2 = math.pi/2.0

        calc = particle.ray_intersect(x1, y1, b1, x2, y2, b2)
        self.assertTrue(calc)

        x1 = 0.0
        y1 = 0.0
        b1 = math.pi/4

        x2 = 1.0
        y2 = 0.0
        b2 = 3*math.pi/4

        calc = particle.ray_intersect(x1, y1, b1, x2, y2, b2)
        self.assertTrue(calc)

        x1 = 0.0
        y1 = 0.0
        b1 = math.pi/4

        x2 = -1.0
        y2 = 0.0
        b2 = 3*math.pi/4

        calc = particle.ray_intersect(x1, y1, b1, x2, y2, b2)
        self.assertFalse(calc)

    def test_add_new_feature(self):
        # TODO(buckbaskin): this is based on other code that needs tested first
        pass

    def test_cross_readings(self):
        particle = FilterParticle()
        old_reading = (Odometry(), Blob(),)
        new_odom = Odometry()
        new_odom.pose.pose.orientation = heading_to_quaternion(math.pi/2)
        new_reading = (new_odom, Blob(),)

        result = particle.cross_readings(old_reading, new_reading)
        self.assertIsNotNone(result)
        self.assertEqual(result[0], 0.0)
        self.assertEqual(result[1], 0.0)

        new_odom = Odometry()
        new_odom.pose.pose.orientation = heading_to_quaternion(0.0)
        new_odom.pose.pose.position.y = -1.0
        new_reading = (new_odom, Blob(),)

        result = particle.cross_readings(old_reading, new_reading)
        self.assertIsNone(result)

    def test_add_orphaned_reading(self):
        particle = FilterParticle()

        original_size = len(particle.hypothesis_set)

        particle.add_orphaned_reading(Odometry(), Blob())

        new_size = len(particle.hypothesis_set)

        self.assertTrue(original_size < new_size)

    # def test_measurement_jacobian(self):
        # TODO(buckbaskin): I'm not sure if I know what to do here to reliably
        #   show if it is correct
        # pass

    # def test_measurement_covariance(self):
        # TODO(buckbaskin): I'm not sure if I know what to do here to reliably
        #   show if it is correct
        # pass

    # def test_measurement_kalman_gain(self):
        # TODO(buckbaskin): I'm not sure if I know what to do here to reliably
        #   show if it is correct
        # pass

    # def test_importance_factor(self):
        # TODO(buckbaskin): I'm not sure if I know what to do here to reliably
        #   show if it is correct
        # pass

    def test_generate_measurement(self):
        particle = FilterParticle()
        odom = Odometry()
        odom.pose.pose.position.x = -1
        odom.pose.pose.position.y = -1

        particle.state = odom

        feature = Feature()
        feature.mean[2] = 73
        feature.mean[3] = 165
        feature.mean[4] = 255

        particle.feature_set[3] = feature

        blob = particle.generate_measurement(3)

        self.assertEqual(feature.mean[2], blob.color.r)
        self.assertEqual(feature.mean[3], blob.color.g)
        self.assertEqual(feature.mean[4], blob.color.b)
        self.assertEqual(blob.bearing, math.pi/4)

class prktFeatureTest(unittest.TestCase):
    def test_initialization(self):
        feature = Feature()
        self.assertIsInstance(feature.mean, np.ndarray)
        self.assertIsInstance(feature.covar, np.ndarray)
        self.assertIsInstance(feature.identity, np.ndarray)
        self.assertEqual(feature.update_count, 0)

if __name__ == '__main__':
    # rospy.init_node('test_node_010a0as12asdjkfobu')
    # rospy.loginfo('sys.version')
    # import sys
    # rospy.loginfo(sys.version.split(' ')[0])

    import rostest
    # rostest.rosrun('crispy_parakeet', 'test_prkt_ros_functionality', RosFunctionalityTest)
    # rostest.rosrun('crispy_parakeet', 'test_prkt_FastSLAM', prktFastSLAMTest)
    # rostest.rosrun('crispy_parakeet', 'test_prkt_Feature', prktFeatureTest)
    rostest.rosrun('crispy_parakeet', 'test_prkt_FilterParticle', prktFilterParticleTest)
    
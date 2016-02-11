#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from utils import heading_to_quaternion, quaternion_to_heading
from viz_feature_sim.msg import Observation

import numpy.random.normal as normal # normal(mu, sigma, count)

class CamSlam360(object):
    '''
    Maintains a state for the robot, as well as for features
    Robot state:
    v1: x, y, heading, v (linear, x), w (angular, z)
    Feature state:
    v1: x, y, r, g, b
    '''
    def __init__(self):
        rospy.init_node('CAMSLAM360')
        # prepare filter

        self.initialize_particle_filter()
        self.initialize_known_landmarks()

        # begin ros updating
        self.cam_sub = rospy.Subscriber('/360cam/features', Observation, 
            self.measurement_update)
        rospy.spin()

    def measurement_update(self, observation):
        pass

if __name__ == '__main__':
    CamSlam360()


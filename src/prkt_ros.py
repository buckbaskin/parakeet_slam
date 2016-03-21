#!/usr/bin/env python

'''
Create a ROS node that uses the parakeet core to do SLAM
'''

import rospy

from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
from prkt_core_v2 import FastSLAM
from viz_feature_sim.msg import VizScan

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

        self.core = None
        self.initialize_particle_filter()

        # begin ros updating
        self.cam_sub = rospy.Subscriber('/360cam/features', VizScan,
            self.measurement_update)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.motion_update)
        rospy.spin()

    def initialize_particle_filter(self):
        '''
        Create an instance of FastSLAM algorithm
        '''
        self.core = FastSLAM()

    def measurement_update(self, msg):
        '''
        Pass along a VizScan message
        '''
        self.core.cam_cb(msg)

    def motion_update(self, msg):
        '''
        Pass along a Twist message
        '''
        self.core.motion_update(msg)


if __name__ == '__main__':
    CamSlam360()


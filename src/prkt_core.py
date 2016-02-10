#!/usr/bin/env python

import rospy
from viz_feature_sim.msg import Observation

class CamSlam360(object):
    def __init__(self):
        rospy.init_node('CAMSLAM360')
        self.cam_sub = rospy.Subscriber('/360cam/features', Observation, self.process_obs)
        rospy.spin()

    def process_obs(self, observation):
        pass

if __name__ == '__main__':
    CamSlam360()


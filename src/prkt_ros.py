#!/usr/bin/env python

'''
Create a ROS node that uses the parakeet core to do SLAM
'''

import rospy

from geometry_msgs.msg import Twist
from matrix import Matrix
# from nav_msgs.msg import Odometry
from prkt_core_v2 import FastSLAM, Feature
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

        preset_covariance = Matrix([[.01,0,0,0,0],
                                    [0,.01,0,0,0],
                                    [0,0,.01,0,0],
                                    [0,0,0,.01,0],
                                    [0,0,0,0,.01]])

        # purple, origin
        feature1 = Feature(mean=Matrix([0,0,161,77,137]), covar=preset_covariance)
        feature1.__immutable__ = True
        # blue
        feature2 = Feature(mean=Matrix([10,0,75,55,230]), covar=preset_covariance)
        feature2.__immutable__ = True
        # green
        feature3 = Feature(mean=Matrix([0,15,82,120,68]), covar=preset_covariance)
        feature3.__immutable__ = True
        # pink
        feature4 = Feature(mean=Matrix([10,15,224,37,192]), covar=preset_covariance)
        feature4.__immutable__ = True

        features = [feature1, feature2, feature3, feature4]

        self.initialize_particle_filter(features)

        # begin ros updating
        self.cam_sub = rospy.Subscriber('/camera/features', VizScan,
            self.measurement_update)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.motion_update)
    
    def run(self):
        if self.core is not None:
            rospy.loginfo('Running!')
            rospy.spin()

    def initialize_particle_filter(self, preset_features):
        '''
        Create an instance of FastSLAM algorithm
        '''
        self.core = FastSLAM(preset_features)

    def measurement_update(self, msg):
        '''
        Pass along a VizScan message
        '''
        self.core.cam_cb(msg)
        self.print_summary()

    def motion_update(self, msg):
        '''
        Pass along a Twist message
        '''
        self.core.motion_update(msg)

    def print_summary(self):
        '''
        average x, y, heading
        '''
        x_sum = 0.0
        y_sum = 0.0
        heading_sum = 0.0
        count = float(len(self.core.particles))

        for particle in self.core.particles:
            x += particle.state.pose.pose.position.x
            y += particle.state.pose.pose.position.y
            heading_sum += quaternion_to_heading(particle.state.pose.pose.orientation)

        x = x_sum / count
        y = y_sum / count
        heading = heading_sum / count
        rospy.loginfo('prkt_summary: '+str((x, y, heading,)))


if __name__ == '__main__':
    cs = CamSlam360()
    cs.run()

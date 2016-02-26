'''
Parakeet-Core

This is the Python module that implements the SLAM algorithm for use in a ROS
node.
'''

# pylint: disable=invalid-name
# pylint: disable=no-self-use

import rospy
import math
import numpy as np

from copy import deepcopy
from geometry_msgs.msg import Twist
from matrix import Matrix, inverse, transpose, mm, identity, magnitude
from nav_msgs.msg import Odometry
from numpy.random import normal
from random import random
from utils import version, heading_to_quaternion, quaternion_to_heading
from viz_feature_sim.msg import VizScan, Blob

class FastSLAM(object):
    def __init__(self):
        self.last_control = Twist()
        self.last_update = rospy.Time.now()
        self.particles = [FilterParticle(), FilterParticle()]
        self.Qt = np.array([[1, 0], [0, 1]]) # measurement noise?

    def cam_cb(self, scan):
        # motion update all particles
        self.motion_update(self.last_control)

        for particle in self.particles:
            particle.weight = 1
            correspondence = particle.match_features_to_scan(scan)
            for pair in correspondence:
                blob = pair[1]
                if pair[0] < 0:
                    # new feature observed
                    particle.add_hypothesis(particle.state, blob)
                    particle.weight *= particle.no_match_weight()
                else:
                    # existing feature observed
                    # generate expected measurement based on feature id
                    pseudoblob = particle.generate_measurement(pair[0])
                    bigH = particle.measurement_jacobian(pair[0])
                    bigQ = particle.measurement_covariance(bigH, pair[0], self.Qt)
                    bigQinv = inverse(bigQ)
                    bigK = particle.kalman_gain(pair[0], bigH, bigQinv)

                    (particle.get_feature_by_id(pair[0])
                        .update_mean(bigK, blob, pseudoblob))
                    (particle.get_feature_by_id(pair[0])
                        .update_covar(bigK, bigH))

                    weighty = particle.importance_factor(bigQ, blob, pseudoblob)
                    particle.weight *= weighty

        self.low_variance_resample()


    def odom_motion_update(self, odom):
        '''
        ***Alpha feature***
        update particles by looking at the change in odometry as the motion
        model
        '''
        pass

    def motion_update(self, new_twist):
        '''
        update the state of all of the particles by the given twist
        input:
            new_twist: twist for the next motion
        output:
            None
        '''
        # TODO(buckbaskin):
        pass

    def low_variance_resample(self):
        '''
        Resample particles based on weights
        '''
        sum_ = reduce(lambda accum, element: accum+element.weight, self.particles, 0.0)
        range_ = sum_/float(len(self.particles))
        step = random()*range_
        temp_particles = []
        count = 0

        ### resample ###

        for particle in self.particles:
            step = step - particle.weight
            while step <= 0.0 and count < len(self.particles):
                # add it to the list
                temp_particles.append(deepcopy(particle))
                # do I need the deepcopy? 
                #   If the motion model creates a new particle, no
                
                # take a step forward
                step += range_
                count += 1
                # repeat if its still in the particle range

        self.particles = temp_particles


class FilterParticle(object):
    def __init__(self, odom=Odometry()):
        self.state = odom
        self.feature_set = {}

    def get_feature_by_id(self, id_):
        '''
        get the feature by id
        currently, if there is no feature for that id, it raises an error
        input:
            int (feature)id_
        output:
            Feature
        raises:
            KeyError
        '''
        return self.feature_set[id_]

    def match_features_to_scan(self, scan):
        '''
        input:
            VizScan scan
        output:
            list of tuples mapping feature ids to Blobs
            negative numbers = new feature
        '''
        # TODO(buckbaskin):
        johndoe = []
        johndoe.append((-1, Blob()))
        johndoe.append((-2, scan.observes[0]))
        return johndoe

    def add_hypothesis(self, state, blob):
        '''
        add a hypothetical new feature to the non-invertable measurement model
        check to see if it matches a hypothetical feature. If that hypothetical
        is strong enough, add it to the particles
        input:
            Odometry state (position of observation)
            Blob blob (observation)
        output:
            None
        '''
        # TODO(buckbaskin):
        pass

    def measurement_jacobian(self, feature_id):
        '''
        Calculate the Jacobian of the measurement model with respect to the
        current particle state and the give feature's mean
        '''
        # TODO(buckbaskin):
        state = self.state
        old_mean = self.get_feature_by_id(feature_id).mean
        return np.array([[0, 0],
                         [0, 0]])

    def measurement_covariance(self, bigH, feature_id, Qt):
        '''
        Calculate the covarance of the measurement with respect to the given
        Jacobian bigH, the feature's covariance and Qt (measurement noise?)
        '''
        # TODO(buckbaskin):
        old_covar = self.get_feature_by_id(feature_id).covar
        return bigH * old_covar * transpose(bigH) + Qt

    def kalman_gain(self, feature_id, bigH, Qinv):
        '''
        Calculate the kalman gain for the update of the feature based on the
        existing covariance of the feature, bigH, and the inverse of bigQ
        '''
        # TODO(buckbaskin):
        old_covar = self.get_feature_by_id(feature_id).covar
        return old_covar * transpose(bigH) * Qinv

    def importance_factor(self, bigQ, blob, pseudoblob):
        # Comment this
        # TODO(buckbaskin):
        return 0.1

class Feature(object):
    def __init__(self):
        # TODO(buckbaskin):
        self.mean = np.array([1, 2, 3, 4, 5])
        self.covar = np.array([[1, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0],
                                [0, 0, 1, 0, 0],
                                [0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 1]])

    def update_mean(self, feature_id, kalman_gain, measure, expected_measure):
        pass

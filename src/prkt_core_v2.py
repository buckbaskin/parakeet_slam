'''
Parakeet-Core

This is the Python module that implements the SLAM algorithm for use in a ROS
node.

# Matrices are rows by columns
'''

# pylint: disable=invalid-name
# pylint: disable=no-self-use

import rospy
import math
import numpy as np

from copy import deepcopy
from geometry_msgs.msg import Twist
from math import sin, cos
from matrix import inverse, transpose, mm, identity, magnitude, madd, msubtract
from matrix import blob_to_matrix, Matrix
from nav_msgs.msg import Odometry
from numpy.random import normal
from random import random
# from scipy.stats import multivariate_normal
from utils import version, heading_to_quaternion, quaternion_to_heading, scale
from utils import dot_product, unit
from viz_feature_sim.msg import VizScan, Blob

# pylint: disable=no-name-in-module
from scipy.stats import multivariate_normal

class FastSLAM(object):
    def __init__(self):
        self.last_control = Twist()
        self.last_update = rospy.Time.now()
        self.particles = [FilterParticle(), FilterParticle()]
        self.Qt = Matrix([[1, 0], [0, 1]]) # measurement noise?

    def cam_cb(self, scan):
        # motion update all particles
        self.motion_update(self.last_control)

        for particle in self.particles:
            particle.weight = 1
            correspondence = particle.match_features_to_scan(scan)
            for pair in correspondence:
                blob = pair[1]
                if pair[0] == 0:
                    # unseen feature observed
                    particle.add_hypothesis(particle.state, blob)
                    particle.weight *= particle.no_match_weight()
                else:
                    # update feature
                    pseudoblob = particle.generate_measurement(pair[0])
                    bigH = particle.measurement_jacobian(pair[0])
                    bigQ = particle.measurement_covariance(bigH, pair[0], self.Qt)
                    bigQinv = inverse(bigQ)
                    bigK = particle.kalman_gain(pair[0], bigH, bigQinv)

                    (particle.get_feature_by_id(pair[0])
                        .update_mean(bigK, blob, pseudoblob))
                    (particle.get_feature_by_id(pair[0])
                        .update_covar(bigK, bigH))
                    if pair[0] < 0:
                        # potential new feature seen
                        # update feature ^ but update as if the feature not seen
                        weighty = particle.no_match_weight()
                        # possibly add the feature to the full feature set
                        if particle.get_feature_by_id(pair[0]).update_count > 5:
                            # the particle has been seen 3 times
                            feature = particle.potential_features[pair[0]]
                            particle.feature_set[-pair[0]] = feature
                            del particle.potential_features[pair[0]]
                    else:
                        # feature seen
                        # update feature and robot pose weight
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
        Input:
            new_twist: twist for the next motion
        Output:
            None
        '''

        twist = self.last_control
        dt = rospy.Time.now() - self.last_update
        for i in range(0, len(self.particles)):
            self.particles[i] = self.motion_model(self.particles[i], self.last_control, dt)

        self.last_update = self.last_update + dt
        self.last_control = new_twist

    def motion_model(self, particle, twist, dt):
        # pS      h1        |
        # 0-----------------0
        # |                 h2
        v = twist.linear.x
        w = twist.angular.z

        new_particle = FilterParticle()
        new_particle.feature_set = deepcopy(particle.feature_set)

        dheading = twist.twist.angular.z * dt

        drive_noise = normal(0, .05*v+.01*w, 1)
        ds = twist.twist.linear.x * dt + drive_noise

        prev_heading = quaternion_to_heading(particle.pose.pose.orientation)

        heading_noise = normal(0, .05*w+.01*v, 1)
        heading_1 = prev_heading+dheading/2+heading_noise

        heading_noise = normal(0, .05*w+.01*v, 1)
        heading_2 = heading_1+dheading/2+heading_noise

        dx = ds*cos(heading_1)
        dy = ds*cos(heading_1)

        new_particle.state.pose.pose.position.x += dx
        new_particle.state.pose.pose.position.y += dy
        new_particle.state.pose.pose.orientation = heading_to_quaternion(heading_2)

        return new_particle

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
    def __init__(self, state=Odometry()):
        self.state = state
        self.feature_set = {}
        self.potential_features = {}

        self.hypothesis_set = {}
        self.next_id = 1

    def get_feature_by_id(self, id_):
        '''
        get the feature by id
        currently, if there is no feature for that id, it raises an error
        Input:
            int (feature)id_
        Output:
            Feature
        raises:
            KeyError
        '''
        if (id_ < 0):
            return self.potential_features[int(id_)]
        else:
            return self.feature_set[id_]

    def match_features_to_scan(self, scan):
        '''
        Version 1: independently match each scan to the most likely feature
        Version 2: match each scan to a unique feature 
            (if they are sufficiently far apart)
        Version 3: graph based enhanced lookup
            Use a graph of features that have been seen together to speed up the
            search for features that are likely to be seen
            Might be: run v1, then take the most likely, and search from there
            for features to match

        ** Note this can match to potential features as well.
            If it matches to a potential feature, the feature will update but
            the weight of the particle will adjust like an unmatched feature

        Input:
            VizScan scan
        Output:
            list of tuples mapping feature ids to Blobs
            <0 = potential new feature
            0 = unseen feature
            >0 = existing full feature

        '''
        # Version 1
        johndoe = []
        for blob in scan.observes:
            johndoe.append((self.match_one(self.state, blob), blob))
        return johndoe

    def match_one(self, state, blob):
        '''
        Return the independent best match to the feature set for the given blob
        Input:
            Odometry state
            Blob blob
        Output:
            int
            <0 = potential new feature
            0 = unseen feature
            >0 = existing full feature
        '''
        # create a list of existing features as (id, feature) pairs
        features = list(self.feature_set.items())
        features.extend(list(self.potential_features.items()))

        max_match = 0.0
        max_match_id = 0

        for id_, feature in features:
            new_match = self.probability_of_match(state, blob, feature)
            if (new_match > max_match):
                max_match = new_match
                max_match_id = id_

        return max_match_id

    def probability_of_match(self, state, blob, feature):
        # TODO(bucbkasin):
        # comment this

        f_mean = feature.mean # [x, y, r, b, g]
        f_covar = feature.covar

        f_x = f_mean[0]
        f_y = f_mean[1]

        s_x = state.pose.pose.position.x
        s_y = state.pose.pose.position.y
        s_heading = quaternion_to_heading(state.pose.pose.orientation)

        expected_bearing = math.atan2(f_y-s_y, f_x-s_x) - s_heading
        observed_bearing = blob.bearing

        del_bearing = observed_bearing - expected_bearing

        color_distance = (math.pow(blob.color.r - f_mean[2], 2) + 
                            math.pow(blob.color.g - f_mean[3], 2) + 
                            math.pow(blob.color.b - f_mean[4], 2))

        if abs(del_bearing) > 0.5:
            return 0.0
        else:
            bearing_prob = self.prob_position_match(f_x, f_y, s_x, s_y, observed_bearing)

        if abs(color_distance) > 300:
            return 0.0
        else:
            color_prob = self.prob_color_match(f_mean, f_covar, blob)

        return bearing_prob*color_prob

    def prob_position_match(self, f_mean, f_covar, s_x, s_y, bearing):
        # TODO(bucbkasin):
        # comment this
        f_x = f_mean[0]
        f_y = f_mean[1]

        # find closest point to feature on the line from state, bearing
        near_x, near_y = self.closest_point(f_x, f_y, s_x, s_y, bearing)
        
        # then use multivariate distribution pdf to find the probability of the
        #   closest point being inside that distribution
        feature_mean = f_mean[0:2]
        feature_covar = f_covar[0:2, 0:2]

        obs_mean = Matrix([near_x, near_y])
        return multivariate_normal.pdf(obs_mean, mean=feature_mean,
            cov=feature_covar)

    def closest_point(self, f_x, f_y, s_x, s_y, obs_bearing):
        '''
        Calculate the closest point on the line to the feature
        The feature is the point (probably not on the line)
        The line is defined by a point (state x and y) and direction (heading)
        Input:
            f_x float (feature's x coordinate)
            f_y float (feature's y coordinate)
            s_x float (robot state's x)
            s_y float (robot state's y)
            obs_bearing float (robot state's heading)
        '''
        # TODO(bucbkasin):

        origin_to_feature = (f_x - s_x, f_y - s_y, 0.0,)
        line_parallel = unit((cos(obs_bearing), sin(obs_bearing), 0.0))

        # origin_to_feature dot line_parallel = magnitude of otf along line
        magmag = dot_product(origin_to_feature, line_parallel)
        scaled_line = scale(line_parallel, magmag)

        scaled_x = scaled_line[0]
        scaled_y = scaled_line[1]

        return (s_x + scaled_x, s_y + scaled_y)

    def prob_color_match(self, f_mean, f_covar, blob):
        # TODO(bucbkasin):
        # comment this

        # TODO(buckbaskin):
        f_r = f_mean[2]
        f_g = f_mean[3]
        f_b = f_mean[4]
        color_mean = Matrix([f_r, f_g, f_b])

        b_r = blob.color.r
        b_g = blob.color.g
        b_b = blob.color.b
        blob_mean = Matrix([b_r, b_g, b_b])

        color_covar = f_covar[2:,2:]

        # use multivariate pdf to calculate the probability of a color match
        return multivariate_normal.pdf(blob_mean, 
            mean=color_mean, cov=color_covar)

    def add_hypothesis(self, state, blob):
        '''
        add a hypothetical new feature to the non-invertable measurement model
        check to see if it matches a hypothetical feature. If that hypothetical
        is strong enough, add it to the particles
        Input:
            Odometry state (position of observation)
            Blob blob (observation)
        Output:
            None
        '''
        pair = self.find_nearest_reading(state, blob)
        if (pair > 0):
            # close enough match to an existing reading
            self.add_new_feature(pair, state, blob)
        else:
            # not a match
            self.add_orphaned_reading(state, blob)

    def find_nearest_reading(self, state, blob):
        '''
        Find the nearest reading in the set of unmatched readings. If it is
        close enough, return the id of the other reading that is close.
        Otherwise, return -1*the id of the other reading that is closest.
        Input:
            Odometry state
            Blob blob
        Output:
            int
        '''
        # TODO(buckbaskin):
        return -1

    def create_new_feature(self, old_id, state, blob):
        '''
        Adds a new feature based on the intersection of the last two readings.
        Input:
            int old_id
            Odometry state
            Blob blob
        Output:
            None
        '''
        new_id = self.next_id
        old_reading = self.hypothesis_set[old_id]
        intersection = self.cross_readings(old_reading, (state, blob,))
        x = intersection[0]
        y = intersection[1]
        r = (old_reading[1].color.r + blob.color.r)/2
        g = (old_reading[1].color.g + blob.color.g)/2
        b = (old_reading[1].color.b + blob.color.b)/2

        mean = Matrix([x, y, r, g, b])
        # TODO(buckbaskin): calculate this covariance
        covar = Matrix([[1,0,0,0,0],
                        [0,1,0,0,0],
                        [0,0,1,0,0],
                        [0,0,0,1,0],
                        [0,0,0,0,1]])
        self.potential_features[-new_id] = Feature(mean=mean, covar=covar)
        self.next_id += 1

    def cross_readings(self, old_reading, new_reading):
        '''
        Find the intersection of the two vectors defined by the position and
        direction of the two readings.
        Returns a tuple of the x, y pair where the two vectors intersect
        Input:
            (Odometry, Blob,) old_reading
            (Odometry, Blob,) new_reading
        Output:
            (float, float)
        '''
        # TODO(buckbaskin):
        return (0.0, 0.0,)

    def add_orphaned_reading(self, state, blob):
        '''
        Add a new reading to the set of orphaned readings that are looking for a
        matching new reading that is close enough to become a potential feature
        '''
        self.hypothesis_set[self.next_id] = ((state, blob,))
        self.next_id += 1

    def measurement_jacobian(self, feature_id):
        '''
        Calculate the Jacobian of the measurement model with respect to the
        current particle state and the give feature's mean

        bigH is the Jacobian of the measurement model with respect to the best
        estimate of the robot's position at the time of the update (computed at 
        the mean of the feature, aka the predicted mean) PR p. 207

        Input:
            int feature_id
        Output:
            np.ndarray 4x3 (bigH)
        '''
        '''
        State = { x, y, heading } = [x , y, heading (theta)]
        measurements = { bearing, r, g, b} = [bearing (phi), r, g, b]

        Jacobian = 
        [[d phi/d x, d phi/d y, d phi/d theta],
         [d r/d x,   d r/d y,   d r/d theta  ],
         [d g/d x,   d g/d y,   d g/d theta  ],
         [d b/d x,   d b/d y,   d b/d theta  ]]
        '''
        state = self.state
        mean_x = state.pose.pose.position.x
        mean_y = state.pose.pose.position.y
        feature_mean = self.get_feature_by_id(feature_id).mean
        feature_x = feature_mean.x
        feature_y = feature_mean.y
        
        # phi = atan2(dy, dx) - heading
        # q = (feature_x - mean_x)^2 + (feature_y - mean_y)^2
        q = pow(feature_x - mean_x, 2) + pow(feature_y - mean_y, 2)

        # d_phi/d_x = (feature_y - mean_y) / q
        d_phi_d_x = (feature_y - mean_y) / q
        
        # d_phi/d_y = (feature_x - mean_x) / q
        d_phi_d_y = (feature_x - mean_x) / q

        # d_phi/d_theta = -1
        d_phi_d_theta = -1.0

        return Matrix([[d_phi_d_x, d_phi_d_y, d_phi_d_theta],
                         [0.0, 0.0, 0.0],
                         [0.0, 0.0, 0.0],
                         [0.0, 0.0, 0.0]])

    def measurement_covariance(self, bigH, feature_id, Qt):
        '''
        Calculate the covarance of the measurement with respect to the given
        Jacobian bigH, the feature's covariance and Qt (measurement noise?)
        Input:
            np.ndarray bigH (Jacobian of the measurement model)
            int feature_id
            np.ndarray Qt (measurement noise)
        Output:
            numpy.ndarray
        '''
        old_covar = self.get_feature_by_id(feature_id).covar
        return Matrix(madd(mm(mm(bigH , old_covar) , transpose(bigH)) , Qt))

    def kalman_gain(self, feature_id, bigH, Qinv):
        '''
        Calculate the kalman gain for the update of the feature based on the
        existing covariance of the feature, bigH, and the inverse of bigQ
        Input:
            int feature_id
            np.ndarray bigH (Jacobian of the measurement model)
            np.ndarray Qinv (inverse of Q, the measurement covariance)
        Output:
            numpy.ndarray
        '''
        old_covar = self.get_feature_by_id(feature_id).covar
        return Matrix(mm(mm(old_covar , transpose(bigH)) , Qinv))

    def importance_factor(self, bigQ, blob, pseudoblob):
        '''
        Calculate the relative importance of this measurement (weight) to be
        used as part of resampling
        Input:
            np.ndarray bigQ (measurement covariance)
            Blob blob (recieved measurement)
            Blob pseudoblob (estimated measurement)
        '''
        v1 = 2*math.pi *magnitude(bigQ)
        v1 = pow(v1, -0.5)
        delz = blob_to_matrix(blob) - blob_to_matrix(pseudoblob)
        delzt = transpose(delz)
        v2 = math.exp(-0.5 * mm(mm(delzt, inverse(bigQ)), delz))
        return v1 * v2

    def no_match_weight(self):
        '''
        return the default weight for when a particle doesn't match an
        observation to an existing feature
        '''
        #TODO(buckbaskin):
        return 0.1

    def generate_measurement(self, featureid):
        '''
        Generate an expected measurement for the given featureid
        '''
        #TODO(buckbaskin): see usage
        return Blob()

    def add_new_feature(self, pair, state, blob):
        # TODO(bucbaskin): see usage
        pass

    # end FilterParticle

class Feature(object):
    def __init__(self, mean=None, covar=None):
        if mean is None:
            mean = Matrix([0, 0, 0, 0, 0])
        if covar is None:
            covar = Matrix([[1, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0],
                                [0, 0, 1, 0, 0],
                                [0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 1]])
        self.mean = mean
        self.covar = covar
        self.identity = identity(covar.shape[0])
        self.update_count = 0

    def update_mean(self, kalman_gain, measure, expected_measure):
        '''
        Update the mean of a known feature based on the calculated Kalman gain
        and the different between the given measurement and the expected
        measurement
        Input:
            np.ndarray kalman_gain
            np.ndarray measure(ment)
            np.ndarray expected_measure(ment)
        Output:
            None
        '''
        delz = blob_to_matrix(measure) - blob_to_matrix(expected_measure)
        adjust = mm(kalman_gain, delz)
        self.mean = self.mean + adjust
        self.update_count += 1

    def update_covar(self, kalman_gain, bigH):
        '''
        Update the covariance of a known feature based on the calculated Kalman
        gain and the Jacobian of the measurement model (bigH)
        Input:
            np.ndarray kalman_gain
            np.ndarray bigH
        Output:
            None
        '''
        adjust = msubtract(self.identity, mm(kalman_gain, bigH))
        self.covar = mm(adjust, self.covar)
        self.update_count += 1

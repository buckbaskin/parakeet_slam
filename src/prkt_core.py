'''
Parakeet-Core

This is the Python module that implements the SLAM algorithm for use in a ROS
node.
'''

# pylint: disable=invalid-name
# pylint: disable=no-self-use

import rospy
import math

from copy import deepcopy
from geometry_msgs.msg import Twist
from matrix import Matrix, inverse, transpose, mm, identity, magnitude
from nav_msgs.msg import Odometry
from random import random
from utils import version, heading_to_quaternion, quaternion_to_heading
from viz_feature_sim.msg import Observation

from numpy.random import normal
# import numpy.random.normal as normal # normal(mu, sigma, count)

class SlamAlgorithm(object):
    '''
    SlamAlgorithm

    This class is a generic class that other algorithms can extend that
    implements common core features, such as the motion model.

    I may eventually add the derivative/Jacobian of the motion model here in
    various forms
    '''
    def __init__(self):
        pass

    def motion_update(self, twist):
        '''
        A method that should be extended for dynamic systems that will update
        the system's position estimate based on a twist-control.

        Inputs:
        twist - ROS Twist message

        Outputs:
        N/A
        '''
        pass

    def motion_model(self, previous_state, twist, dt):
        '''
        A method that returns a noisy (default) version of the motion model for
        evolving a previous state based on a twist and a time step

        Inputs:
        previous_state - ROS Odometry-like
        twist - ROS Twist message
        dt - a time delta

        Outputs:
        next_state - ROS Odometry-like
        '''
        return self.motion_model_noisy(previous_state, twist, dt)

    def motion_model_noisy(self, previous_state, twist, dt):
        '''
        A method that returns a noisy (default) version of the motion model for
        evolving a previous state based on a twist and a time step

        Inputs:
        previous_state - ROS Odometry-like
        twist - ROS Twist message
        dt - a time delta

        Outputs:
        next_state - ROS Odometry-like
        '''
        # pS      h1        |
        # 0-----------------0
        # |                 h2
        dheading = twist.twist.angular.z * dt

        drive_noise = normal(0, .05, 1)
        ds = twist.twist.linear.x * dt + drive_noise

        # prev_x = previous_state.pose.pose.position.x
        # prev_y = previous_state.pose.pose.position.y
        # pylint: disable=line-too-long
        prev_heading = quaternion_to_heading(previous_state.pose.pose.orientation)

        heading_noise = normal(0, .05, 1)
        heading_1 = prev_heading+dheading/2+heading_noise

        heading_noise = normal(0, .05, 1)
        heading_2 = heading_1+dheading/2+heading_noise

        dx = ds*math.cos(heading_1)
        dy = ds*math.cos(heading_1)

        previous_state.pose.pose.position.x += dx
        previous_state.pose.pose.position.y += dy
        previous_state.pose.pose.orientation = heading_to_quaternion(heading_2)

        return previous_state

class ParticleMixedSlam(SlamAlgorithm):
    '''
    Need to combine 3 measurements to get an initial estimate for the pose of
     the landmark. Use this in combination with maintaining new landmark
     hypothesis to start adding landmarks. When they are ready to be added,
     initialize a gaussian based on the first triangulation. Then use this to
     start measuring the likelihood of a new measurement corresponding to that
     landmark

    To trim down the number of landmarks that need to be scanned, create a graph
     of landmarks. On this graph, connect landmarks that have been seen together
     before. When searching for landmarks, do a graph traversal starting from
     the initial pose out, stopping at some number of nodes on the graph.
     Potentially weight edges with the number of times that two features have
     been seen together

    In small cases, this will just search all of the nodes, eventually with lots
     of data it should limit the correspondence to new nodes and nodes that are
     close together
    '''

    def __init__(self):
        # particle = individiual state estimate + 1 kalman filter per map
        #   feature (total of N)
        # Particle filter: M particles
        super(ParticleMixedSlam, self).__init__()
        self.M = 10
        self.robot_particles = []
        self.initialize_particle_filter()

        self.Qt = Matrix([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])

        self.hypothesis_features = []

        self.last_time = rospy.Time.now()
        self.last_twist = Twist()

    @version(0, 2, 0)
    def initialize_particle_filter(self):
        '''
        create all of the initial guesses for the robot location based on a
        coded-in distribution
        Input:
        N/A
        Output:
        N/A
        '''
        for _ in range(0, int(self.M)):
            self.robot_particles.append(RobotParticle(0, 0.1))

    @version(0, 2, 0)
    def motion_update(self, twist):
        '''
        Based on a given twist (think ROS callback), evolve the particle list
        Input:
        twist
        Output:
        N/A
        '''
        new_time = rospy.Time.now()
        dt = new_time - self.last_time
        for particle in self.robot_particles:
            # pylint: disable=line-too-long
            particle.state = particle.motion_model(particle.state, self.last_twist, dt)
        self.last_time = new_time
        self.last_twist = twist

    @version(0, 2, 0)
    def measurement_update(self, measurement):
        '''
        Evolve the particle list based on a measurement (think ROS callback)
        '''
        if isinstance(measurement, Observation):
            self.cam_observation_update(measurement)
        else:
            pass

    # @version(0, 2, 0)
    # def condense_robot_state(self):
    #     avg_state = 0

    @version(0, 2, 0)
    def cam_observation_update(self, cam_obs):
        '''Single bearing-color observation'''
        zt = Matrix([cam_obs.bearing,
            cam_obs.color.r, cam_obs.color.g, cam_obs.color.b])

        self.motion_update(self.last_twist)

        for particle in self.robot_particles:
            j = particle.get_feature_id(zt)
            if j < 0: # not seen before
                # pylint: disable=line-too-long
                new_mean = self.inverse_cam_measurement_model(particle.state, zt)
                H = self.jacobian_of_motion_model(particle.state, new_mean)
                H_inverse = inverse(H)
                new_covar = mm(mm(H_inverse, self.Qt), transpose(H_inverse))
                particle.add_new_feature_ekf(new_mean, new_covar)
                particle.weight = particle.default_weight()
            else: # j seen before
                feature = particle.get_feature_by_id(j)
                # pylint: disable=line-too-long
                # naming explains functionality
                z_hat = particle.measurement_prediction(feature.mean, particle.state)
                H = self.jacobian_of_motion_model(particle.state, feature.mean)
                Q = mm(mm(H, feature.covar), transpose(H)) + self.Qt
                Q_inverse = inverse(Q)
                K = mm(mm(feature.covar, transpose(H)), Q_inverse)

                new_mean = feature.mean + mm(K, zt - z_hat)
                new_covar = mm(identity(5) - mm(K, H), feature.covar)

                particle.replace_feature_ekf(j, new_mean, new_covar)
                particle.weight = pow(2*math.pi*magnitude(Q), -1/2) * math.exp(-0.5 * (transpose(zt - z_hat)*Q_inverse*(zt - z_hat)))
            # endif
            # for all other features...do nothing
        # end for

        temp_particle_list = []
        sum_ = 0
        for particle in self.robot_particles:
            sum_ = sum_ + particle.weight

        chosen = random()*sum_

        for _ in range(0, len(self.robot_particles)):
            for particle in self.robot_particles:
                chosen = chosen - particle.weight
                if chosen < 0:
                    # choose this particle
                    temp_particle_list.append(particle.deep_copy())

        self.robot_particles = temp_particle_list

    def inverse_cam_measurement_model(self, state, measurement):
        '''
        Based on an estimated state and measurement, return the estimated pose
        pose of the measured object
        '''
        # TODO(buckbaskin):
        # returns mean for a new measurement ekf (x,y,r,g,b)
        return Matrix([0, 0, 0, 0, 0])

    def jacobian_of_motion_model(self, state, mean):
        '''
        returns the partial derivative of the motion model with respect to the
        components of the state
        '''
        # TODO(buckbaskin):
        # self.jacobian_of_motion_model(particle.state, new_mean)
        # returns an H matrix
        return Matrix([[0, 1], [2, 3]])

@version(0, 2, 0)
class RobotParticle(Odometry):
    '''
    Represents a particle for FastSLAM
    '''
    def __init__(self, mean, distribution):
        super(RobotParticle, self).__init__()
        self.pose.pose.position.x = mean + normal(0, distribution)
        self.pose.pose.position.y = mean + normal(0, distribution)

        self.feature_models = []
        self.N = len(self.feature_models)
        self.weight = 0
        self.state = self

    def add_new_feature_ekf(self, mean, covar):
        '''
        Add a new feature to the particle's map, given the mean and covariance
        to start with
        '''
        # TODO(buckbaskin):
        pass

    def replace_feature_ekf(self, feature_id, mean, covar):
        '''
        Replace/update an existing feature with a new mean and covariance
        '''
        # TODO(buckbaskin):
        pass

    def get_feature_id(self, measurement):
        '''
        get the index in the list of the feature that the measurement aligns
        with. If the feature doesn't align, return a negative number to indicate
        an unseen feature
        '''
        # TODO(buckbaskin): may replace this method
        # for now, return < 0 for a new particle
        pass

    def get_feature_by_id(self, feature_id):
        '''get the feature at the given index'''
        # TODO(buckbaskin): may replace this method
        pass

    def default_weight(self):
        '''default weight for new particles in resampling'''
        # TODO(buckbaskin):
        return 0.5

    def deep_copy(self):
        '''deep copy the particle for resampling'''
        return deepcopy(self)


class FeatureModel(object):
    '''
    Model/EKF for representing the pose and color of map features
    '''
    def __init__(self, mean, covar):
        '''
        gaussian model of a feature state (x, y, r, g, b)
        '''
        self.x = mean[0]
        self.y = mean[1]
        self.r = mean[2]
        self.g = mean[3]
        self.b = mean[4]
        self.mean_array = mean[0:5]
        for i in range(0, 5):
            covar[i] = covar[i][0:5]
        self.covar_matrix = covar[0:5]

    def mean(self):
        '''get the best estimate state of the EKF'''
        return [self.x, self.y, self.r, self.g, self.b]

    def covar(self):
        '''get the covariance of the EKF'''
        return self.covar_matrix

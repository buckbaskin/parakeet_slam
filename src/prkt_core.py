import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from utils import version
from viz_feature_sim.msg import Observation

import numpy.random.normal as normal # normal(mu, sigma, count)

class SlamAlgorithm(object):
    def __init__(self):
        pass

    def measurement_update(self, measurement):
        pass

    def measurement_model(self):
        pass

    def motion_update(self, twist):
        pass

    def motion_model(self, prevState, twist, dt):
        return self.motion_model_noisy(prevState, twist, dt)

    def motion_model_noisy(self, prevState, twist, dt):
        # pS      h1        |
        # 0-----------------0
        # |                 h2
        dheading = twist.twist.angular.z * dt
        
        drive_noise = normal(0, .05, 1)
        ds = twist.twist.linear.x * dt

        prev_x = prevState.pose.pose.position.x
        prev_y = prevState.pose.pose.position.y
        prev_heading = quaternion_to_heading(prevState.pose.pose.orientation)

        heading_noise = normal(0, .05, 1)
        heading_1 = prev_heading+dheading/2+heading_noise

        heading_noise = normal(0, .05, 1)
        heading_2 = heading_1+dheading/2+heading_noise

        dx = ds*math.cos(heading_1)
        dy = ds*math.cos(heading_1)

        prevState.pose.pose.position.x += dx
        prevState.pose.pose.position.y += dy
        prevState.pose.pose.orientation = heading_to_quaternion(heading_2)

        return prevState

    def motion_model_noise_free(self, prevState, twist, dt):
        # for reference
        # pS      h1        |
        # 0-----------------0
        # |                 h2
        dheading = twist.twist.angular.z * dt
        ds = twist.twist.linear.x * dt
        prev_x = prevState.pose.pose.position.x
        prev_y = prevState.pose.pose.position.y
        prev_heading = quaternion_to_heading(prevState.pose.pose.orientation)

        heading_1 = prev_heading+dheading/2
        heading_2 = prev_heading+dheading
        dx = ds*math.cos(heading_1)
        dy = ds*math.cos(heading_1)

        prevState.pose.pose.position.x += dx
        prevState.pose.pose.position.y += dy
        prevState.pose.pose.orientation = heading_to_quaternion(heading_2)

        return prevState

class ParticleMixedSlam(SlamAlgorithm):
    '''
    Need to combine 3 measurements to get an initial estimate for the pose of the landmark
     Use this in combination with maintaining new landmark hypothesis to start adding landmarks
     When they are ready to be added, initialize a gaussian based on the first triangulation
     Then use this to start measuring the likelihood of a new measurement corresponding to that landmark

    To trim down the number of landmarks that need to be scanned, create a graph of landmarks
     On this graph, connect landmarks that have been seen together before. When searching for landmarks,
     do a graph traversal starting from the initial pose out, stopping at some number of nodes on the graph
     Potentially weight edges with the number of times that two features have been seen together

    In small cases, this will just search all of the nodes, eventually with lots of data it should limit
     the correspondence to new nodes and nodes that are close together
    '''

    def __init__(self):
        self.robot_particles = []
        self.feature_models = []
        self.hypothesis_features = []
        self.last_time = rospy.Time.now()
        self.last_twist = Twist()

    @version(1)
    def measurement_update(self, measurement):
        if is_instance(measurement, Observation):
            self.cam_observation_update(measurement)
        else:
            pass

    @version(1)
    def condense_robot_state(self):
        avg_state = 0

    def cam_observation_model(self, cam_obs):
        '''Single bearing-color observation'''
        
        self.motion_model(self.last_twist)
        state = self.condense_robot_state()

        x = state.pose.pose.position.x
        y = state.pose.pose.position.y
        robot_heading = quaternion_to_heading(state.pose.pose.orientation)
        map_orientation = robot_heading + cam_obs.bearing

        feature_matches = []

        for feature in self.feature_models:
            feature_matches.append(
                (self.prob_feature_match(x, y, map_orientation, feature), 
                    feature)
                )

        max_val = 0.0
        index = 0
        for i in range(0, len(feature_matches)):
            if feature_matches[i][0] > max_val:
                index = i

        # matched the measurement to the most likely feature

        # check probability of a new feature
        new_chance = self.prob_new_feature()
        if new_chance > max_val:
            # then I probably saw a new feature
            # add this to the hypothesis
            # update map without updating robot location?
            self.add_hypothesis(state, cam_obs)
            # TODO(bucbkasin): finish the rest of this update
        else:
            # update that feature and the map
            # TODO(bucbkasin)
            max_feature = self.feature_models[index]


    def prob_feature_match(self, x, y, m_o, feature):
        # TODO(bucbkasin)
        return 0.5

    def prob_new_feature(self, x, y, m_o):
        # TODO(bucbkasin)
        return 0.45

    def add_hypothesis(self, state, cam_obs):
        # check entire existing list for similar colors
        # if there are 2 or more existing hypothesis that match well enough
        # create a new map feature and pop those two off the list
        # else, add hypothesis to the list

        # TODO(bucbkasin)
        pass

    def motion_update(self, twist):
        self.last_twist = twist
        dt = rospy.Time.now() - self.last_time
        for i in range(0, len(self.robot_particles)):
            self.robot_particles[i] = motion_model_noisy(self.robot_particles[i], twist, dt)
        self.last_time = self.last_time + dt

    def initialize_particle_filter(self, size):
        self.robot_particles = []
        for _ in range(0,size):
            o = RobotParticle()
            o.pose.pose.position.x = normal(0,1,1)
            o.pose.pose.position.y = normal(0,1,1)
            o.pose.pose.position.z = 0
            o.pose.pose.orientation = heading_to_quaternion(normal(0,1,0))
            o.twist.twist.linear.x = normal(0,1,1)
            o.twist.twist.linear.y = 0
            o.twist.twist.linear.z = 0
            o.twist.twist.angular.x = 0
            o.twist.twist.angular.y = 0
            o.twist.twist.angular.z = normal(0,1,1)
            self.robot_particles.append(o)

    def initialize_known_landmarks(self):
        self.landmarks = []

class RobotParticle(Odometry):
    pass

class FeatureModel(object):
    def __init__(x, x_sigma, y, y_signma, r, r_sigma, g, g_sigma, b, b_sigma):
        '''
        gaussian model of a feature state (x, y, r, g, b)
        '''
        self.x = x
        self.x_sigma = x_sigma
        self.y = y
        self.y_sigma = y_sigma
        self.r = r
        self.r_sigma = r_sigma
        self.g = g
        self.g_sigma = g_sigma
        self.b = b
        self.b_sigma = b_sigma
        
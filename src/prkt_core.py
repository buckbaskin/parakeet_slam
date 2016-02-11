import rospy

from nav_msgs.msg import Odometry

import numpy.random.normal as normal # normal(mu, sigma, count)

class SlamAlgorithm(object):
    def __init__(self):
        pass

    def measurement_update(self):
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
        self.last_time = rospy.Time.now()

    def measurement_update(self):
        pass

    def measurement_model(self):
        pass

    def motion_update(self, twist):
        dt = rospy.Time.now() - self.last_time
        for i in range(0, len(self.robot_particles)):
            self.robot_particles[i] = motion_model_noisy(self.robot_particles[i], twist, dt)

    def initialize_particle_filter(self, size):
        self.robot_particles = []
        for _ in range(0,size):
            o = Odometry()
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
        
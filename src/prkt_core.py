class SlamAlgoritm(object):
    def __init__(self):
        pass

    def measurement_update(self):
        pass

    def measurement_model(self):
        pass

    def motion_update(self, twist):
        pass

    def motion_model(self, twist):
        self.motion_model_noise_free(self, twist)

    def motion_model_noise_free(self, twist):
        pass

class ParticleMixedSlam(SlamAlgoritm):
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
'''
Parakeet-Core

This is the Python module that implements the SLAM algorithm for use in a ROS
node.

# Matrices are rows by columns
'''

# pylint: disable=invalid-name
# pylint: disable=fixme
# pylint: disable=no-self-use

import rospy

import copy as copy_module
import math
import sys
# import numpy as np

from copy import deepcopy
from geometry_msgs.msg import Twist
from math import sin, cos
from matrix import inverse, mm, identity, magnitude, madd, msubtract
from matrix import blob_to_matrix, Matrix
from nav_msgs.msg import Odometry
from numpy.random import normal
from random import random
# from scipy.stats import multivariate_normal
from utils import heading_to_quaternion, quaternion_to_heading, scale
from utils import dot_product, unit
from viz_feature_sim.msg import Blob

# pylint: disable=no-name-in-module
from scipy.stats import multivariate_normal

class FastSLAM(object):
    def __init__(self, preset_features=[]):
        self.last_control = Twist()
        self.last_update = rospy.Time.now()
        self.num_particles = 50
        self.particles = [None]*self.num_particles
        for i in range(0,self.num_particles):
            self.particles[i] = FilterParticle()
        
        for particle in self.particles:
            if rospy.is_shutdown():
                break
            particle.load_feature_list(preset_features)
        self.Qt = Matrix([[.1, 0, 0, 0], 
                          [0, .1, 0, 0],
                          [0, 0, .1, 0],
                          [0, 0, 0, .1]]) # measurement noise

        self.aged_particles_pub = rospy.Publisher('/aged_particles', Odometry, queue_size=1)
        self.resampled_particles_pub = rospy.Publisher('/resampled_particles', Odometry, queue_size=1)
        self.particle_track_pub = rospy.Publisher('/particle_track', Odometry, queue_size=1)

    def cam_cb(self, ros_view):
        # motion update all particles

        rospy.loginfo('rolling cam_cb')
        rospy.loginfo('core_v2: cam_cb -> pre low_variance_resample')

        count = 0

        for i in range(0, len(self.particles)):
            if rospy.is_shutdown():
                break
            count += 1
            if (count % 10) == 0:
                rospy.loginfo('particle: %d' % count)
            self.particles[i].weight = 1

            if count == 1:
                rospy.loginfo('<<< start motion_update %d' % count)
                self.motion_update(self.last_control)

            if (count % 10) == 0:
                rospy.loginfo('<<< start correspondence %d' % count)

            scan = ros_view.last_sensor_reading

            correspondence = self.particles[i].match_features_to_scan(scan)
            if (count % 10) == 0:
                rospy.loginfo('<<< end correspondence %d' % count)
            
            for pair in correspondence:
                if rospy.is_shutdown():
                    break
                blob = pair[1]
                if pair[0] == 0:
                    # unseen feature observed
                    self.particles[i].add_hypothesis(self.particles[i].state, blob)
                    self.particles[i].weight *= self.particles[i].no_match_weight()
                else:
                    # update feature
                    pseudoblob = self.particles[i].generate_measurement(pair[0])
                    bigH = self.particles[i].measurement_jacobian(pair[0])
                    # pylint: disable=line-too-long
                    bigQ = self.particles[i].measurement_covariance(bigH, pair[0], self.Qt)
                    bigQinv = inverse(bigQ)
                    bigK = self.particles[i].kalman_gain(pair[0], bigH, bigQinv)

                    (self.particles[i].get_feature_by_id(pair[0])
                        .update_mean(bigK, blob, pseudoblob))
                    (self.particles[i].get_feature_by_id(pair[0])
                        .update_covar(bigK, bigH))
                    if pair[0] < 0:
                        # potential new feature seen
                        # update feature ^ but update as if the feature not seen
                        weighty = self.particles[i].no_match_weight()
                        # possibly add the feature to the full feature set
                        if self.particles[i].get_feature_by_id(pair[0]).update_count > 5:
                            # the self.particles[i] has been seen 3 times
                            feature = self.particles[i].potential_features[pair[0]]
                            self.particles[i].feature_set[-pair[0]] = feature
                            del self.particles[i].potential_features[pair[0]]
                    else:
                        # feature seen
                        # update feature and robot pose weight
                        # pylint: disable=line-too-long
                        weighty = self.particles[i].importance_factor(bigQ, blob, pseudoblob)
                    self.particles[i].weight *= weighty
            
            self.particles[i].state.header.frame_id = 'odom'
            self.particle_track_pub.publish(self.particles[i].state)
            
            if abs(self.particles[i].weight - 1) < .001:
                rospy.loginfo('suspicious 1: %d' % len(correspondence))
            else:
                rospy.loginfo('not suspicious weight: %f' % (self.particles[i].weight,))
            if (count % 10) == 0:
                rospy.loginfo('<<< end correspondence loop %d' % count)

        rospy.loginfo('core_v2: cam_cb -> post low_variance_resample')
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
        # rospy.loginfo('core_v2: motion_update '+str(new_twist))
        # rospy.loginfo('time: '+str(rospy.Time.now())+' | '+str(self.last_update))
        dt = rospy.Time.now() - self.last_update
        for i in xrange(0, len(self.particles)):
            if rospy.is_shutdown():
                break
            self.particles[i] = self.motion_model(self.particles[i],
                self.last_control, dt)

        self.last_update = self.last_update + dt
        self.last_control = new_twist

    def motion_model(self, particle, twist, dt):
        # pS      h1        |
        # 0-----------------0
        # |                 h2
        # dt = dt.secs + dt.nsecs*math.pow(10,-9)
        # rospy.loginfo('dt secs: '+str(dt.to_sec()))
        dt = dt.to_sec()

        v = twist.linear.x
        w = twist.angular.z

        new_particle = FilterParticle()
        
        new_particle = copy_module.deepcopy(particle)

        dheading = twist.angular.z * dt

        drive_noise = normal(0, abs(.05*v)+abs(.005*w)+.0005, 1)
        ds = twist.linear.x * dt + drive_noise

        prev_heading = quaternion_to_heading(particle.state.pose.pose.orientation)

        heading_noise = normal(0, abs(.025*w)+abs(.005*v)+.0005, 1)
        heading_1 = prev_heading+dheading/2+heading_noise

        heading_noise = normal(0, abs(.025*w)+abs(.005*v)+.0005, 1)
        heading_2 = heading_1+dheading/2+heading_noise

        # rospy.loginfo('asdf;kjasdf; '+str(heading_1 ))

        dx = ds*cos(heading_1)
        dy = ds*sin(heading_1)

        # rospy.loginfo('ds delta: '+str(ds - v*dt))

        new_particle.state.pose.pose.position.x += dx
        new_particle.state.pose.pose.position.y += dy
        # pylint: disable=line-too-long
        new_particle.state.pose.pose.orientation = heading_to_quaternion(heading_2)

        return new_particle

    def low_variance_resample(self):
        '''
        Resample particles based on weights
        '''
        rospy.loginfo('low_variance_resample()')

        sum_ = 0
        max_ = 0
        for element in self.particles:
            # rospy.loginfo(element.weight)
            sum_ += element.weight
            if element.weight > max_:
                max_ = element.weight

        rospy.loginfo('summmm_ %f %f' % (sum_, max_,))
        range_ = sum_/float(len(self.particles))
        step = random()*range_
        temp_particles = []
        count = 0

        rospy.loginfo('reshample')
        ### resample ###
        
        for particle in self.particles:
            if rospy.is_shutdown():
                break
            particle.state.header.frame_id = 'odom'
            self.aged_particles_pub.publish(particle.state)
            step = step - particle.weight
            while step <= 0.0 and count < len(self.particles):
                # add it to the list
                particle.state.header.frame_id = 'odom'
                self.resampled_particles_pub.publish(particle.state)
                temp_particles.append(copy_module.deepcopy(particle))
                # do I need the deepcopy?
                #   If the motion model creates a new particle, no

                # take a step forward
                step += range_
                count += 1
                # repeat if its still in the particle range

        self.particles = temp_particles

    def summary(self):
        '''
        average x, y, heading
        '''
        x_sum = 0.0
        y_sum = 0.0
        heading_dx = 0.0
        heading_dy = 0.0
        count = float(len(self.particles))

        for particle in self.particles:
            if rospy.is_shutdown():
                break
            x_sum += float(particle.state.pose.pose.position.x)
            y_sum += float(particle.state.pose.pose.position.y)
            heading_t = float(quaternion_to_heading(particle.state.pose.pose.orientation))
            heading_dx += cos(heading_t)
            heading_dy += sin(heading_t)

        x = x_sum / count
        y = y_sum / count
        heading = math.atan2(heading_dy, heading_dx)
        return (x, y, heading,)

class FilterParticle(object):
    def __init__(self, state=None):
        if state is None:
            state = Odometry()
            state.pose.pose.position.x = 0.0
            state.pose.pose.position.y = 0.0
            state.pose.pose.orientation = heading_to_quaternion(0.0)
        self.state = state
        self.feature_set = {}
        self.potential_features = {}
        self.weight = 1


        self.hypothesis_set = {}
        self.next_id = 1

    def load_feature_list(self, features):
        for feature in features:
            if rospy.is_shutdown():
                break
            self.feature_set[self.next_id] = feature
            self.next_id += 1

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
        if id_ < 0:
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
        count = 0
        for blob in scan.observes:
            if rospy.is_shutdown():
                break
            count += 1
            # rospy.loginfo('start johndoe %d' % count)
            johndoe.append((self.match_one(self.state, blob), blob))
            # rospy.loginfo('end   johndoe %d' % count)
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
            if rospy.is_shutdown():
                break
            new_match = self.probability_of_match(state, blob, feature)
            # rospy.loginfo('%d new match %f' % (id_, new_match))
            if new_match > max_match:
                max_match = new_match
                max_match_id = id_

        return max_match_id

    def probability_of_match(self, state, blob, feature):
        '''
        Calculate the probability that a state and a blob observation match a
        given feature

        Input:
            Odometry state
            Blob blob
            Feature feature
        Output:
            float (probability)
        '''

        f_mean = feature.mean # [x, y, r, b, g]
        f_covar = feature.covar

        f_x = f_mean[0]
        f_y = f_mean[1]

        s_x = state.pose.pose.position.x
        s_y = state.pose.pose.position.y
        s_heading = quaternion_to_heading(state.pose.pose.orientation)

        # rospy.loginfo('atan2(%f - %f, %f - %f) - %f' % (f_y, s_y, f_x, s_x, s_heading,))

        expected_bearing = math.atan2(f_y-s_y, f_x-s_x) - s_heading
        observed_bearing = blob.bearing

        # rospy.loginfo('%f vs. %f' % (expected_bearing, observed_bearing,))
        # rospy.loginfo('feature mean: %s' % str(f_mean))
        # rospy.loginfo('state (%f, %f, %f,) and blob %f, size(%f)' % (s_x, s_y, s_heading, observed_bearing, blob.size))

        del_bearing = observed_bearing - expected_bearing
        # while(del_bearing > math.pi*2):
        #     del_bearing = del_bearing - math.pi*2
        # while(del_bearing < math.pi*-2):
        #     del_bearing = del_bearing + math.pi*2
        # if(del_bearing > math.pi):
        #     del_bearing = math.pi*-2 + del_bearing
        # if(del_bearing < -math.pi):
        #     del_bearing = math.pi*2 + del_bearing

        color_distance = (math.pow(blob.color.r - f_mean[2], 2) +
                            math.pow(blob.color.g - f_mean[3], 2) +
                            math.pow(blob.color.b - f_mean[4], 2))

        # add the 500s to boost small numbers away from 0. Everything gets the
        #   multiple, but it should make the multiplication of the two numbers
        #   less likely to hit 0 unless one of them really is 0

        if abs(del_bearing) > 0.5:
            # rospy.loginfo('color distance was ... %f' % color_distance)
            # rospy.loginfo('bearing exit')
            return 0.0
        else:
            # pylint: disable=line-too-long
            bearing_prob = 500.0*self.prob_position_match(f_mean, f_covar, s_x, s_y, observed_bearing)

        if abs(color_distance) > 300:
            # rospy.loginfo('%d %d %d | %d %d %d' % (blob.color.r, blob.color.g, blob.color.b, f_mean[2], f_mean[3], f_mean[4]))
            # rospy.loginfo('color exit')
            return 0.0
        else:
            color_prob = 500.0*self.prob_color_match(f_mean, f_covar, blob)
            # rospy.loginfo('bp: %f' % bearing_prob)
            # rospy.loginfo('cp: %f' % color_prob)

        if not isinstance(bearing_prob, float):
            rospy.loginfo('type(bearing_prob) %s' % str(type(bearing_prob)))
        if not isinstance(color_prob, float):
            rospy.loginfo('type(color_prob) %s' % str(type(color_prob)))
            
        return bearing_prob*color_prob / (250000.0)

    def prob_position_match(self, f_mean, f_covar, s_x, s_y, bearing):
        '''
        Calculate the probability that the feature's position matches the
        state's position and observed bearing
        Input:
            np.ndarray f_mean
            np.ndarray f_covar
            float s_x
            float s_y
            float bearing
        Output:
            float
        '''
        f_x = float(f_mean[0])
        f_y = float(f_mean[1])

        pse_bearing = math.atan2(f_y-s_y, f_x-s_x)
        if abs(pse_bearing - bearing) > math.pi/2:
            return 0.0

        # find closest point to feature on the line from state, bearing
        near_x, near_y = self.closest_point(f_x, f_y, s_x, s_y, bearing)

        # then use multivariate distribution pdf to find the probability of the
        #   closest point being inside that distribution
        feature_mean = Matrix([f_x, f_y])
        feature_mean.flatten()
        
        feature_covar = f_covar[0:2, 0:2]

        obs_mean = Matrix([near_x, near_y])
        obs_mean.flatten()
        result = multivariate_normal.pdf(obs_mean, mean=feature_mean,
            cov=feature_covar)
        if not isinstance(result, float):
            rospy.loginfo('scipy gotcha: %s' % str(type(result)))
            rospy.loginfo('result: %s' % str(result))
        return result

    def closest_point(self, f_x, f_y, s_x, s_y, obs_bearing):
        '''
        Calculate the closest point on the line to the feature
        The feature is the point (probably not on the line)
        The line is defined by a point (state x and y) and direction (heading)
        This probably won't return a point that is behind the x-y-bearing.
        Input:
            f_x float (feature's x coordinate)
            f_y float (feature's y coordinate)
            s_x float (robot state's x)
            s_y float (robot state's y)
            obs_bearing float (robot state's heading)
        '''
        origin_to_feature = (f_x - s_x, f_y - s_y, 0.0,)
        line_parallel = unit((cos(obs_bearing), sin(obs_bearing), 0.0))

        # origin_to_feature dot line_parallel = magnitude of otf along line
        magmag = dot_product(origin_to_feature, line_parallel)
        
        if magmag < 0:
            return (s_x, s_y)
        
        scaled_line = scale(line_parallel, magmag)
        scaled_x = scaled_line[0]
        scaled_y = scaled_line[1]

        return (float(s_x + scaled_x), float(s_y + scaled_y))

    def prob_color_match(self, f_mean, f_covar, blob):
        '''
        Calculate the likelihood of the observed color being within the
        distribution defined by the color mean and covariance from the feature
        '''

        f_r = f_mean[2]
        f_g = f_mean[3]
        f_b = f_mean[4]
        color_mean = Matrix([f_r, f_g, f_b])

        b_r = blob.color.r
        b_g = blob.color.g
        b_b = blob.color.b
        blob_mean = Matrix([b_r, b_g, b_b])

        color_covar = f_covar[2:, 2:]

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
        pair_id = self.find_nearest_reading(state, blob)
        if pair_id > 0:
            # close enough match to an existing reading
            self.add_new_feature(pair_id, state, blob)
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
        min_dist_id = 0
        min_dist = float('inf')

        for id_, reading in self.potential_features.iteritems():
            if rospy.is_shutdown():
                break
            reading_state = reading[0]
            reading_blob = reading[1]
            d = self.reading_distance_function(reading_state, reading_blob,
                state, blob)
            if d < min_dist:
                min_dist = d
                min_dist_id = id_

        return min_dist_id

    def reading_distance_function(self, state1, blob1, state2, blob2):
        '''
        Find a distance between two state-blob rays with colors. If they rays do
        not intersect, the distance is infinite. Otherwise, it is the distance
        between two colors.
        '''
        x1 = state1.pose.pose.position.x
        y1 = state1.pose.pose.position.y
        b1 = blob1.bearing + quaternion_to_heading(state1.pose.pose.orientation)
        x2 = state2.pose.pose.position.x
        y2 = state2.pose.pose.position.y
        b2 = blob2.bearing + quaternion_to_heading(state2.pose.pose.orientation)

        if not self.ray_intersect(x1, y1, b1, x2, y2, b2):
            return float('inf')

        return self.color_distance(blob1, blob2)

    def ray_intersect(self, x1, y1, b1, x3, y3, b3):
        '''
        Given the (x,y) pair and bearing of two rays, find out if they intersect
        with a reduced form of the calculation from cross_readings

        stackoverflow.com/questions/2931573/determining-if-two-rays-intersect
        stackoverflow.com/questions/2931573/2353236

        Input:
            float x1, y1, b1, x3, y3, b3
        Output:
            boolean
        '''
        as_ = (x1, y1,)
        ad_ = (cos(b1), sin(b1))
        bs_ = (x3, y3,)
        bd_ = (cos(b3), sin(b3))

        if (ad_[1]*bd_[0]-ad_[0]*bd_[1]) == 0:
            return False

        v = ((ad_[0]*bs_[1] - ad_[1]*bs_[0] + ad_[1]*as_[0] - ad_[0]*as_[1]) / 
            (ad_[1]*bd_[0]-ad_[0]*bd_[1]))

        if (abs(ad_[1]) < abs(ad_[0])):
            u = (bs_[0] + bd_[0]*v-as_[0])/(ad_[0])
        else:
            u = (bs_[1] + bd_[1]*v-as_[1])/(ad_[1])


        return u >= 0 and v >= 0

    def color_distance(self, blob1, blob2):
        r1 = blob1.color.r
        r2 = blob2.color.r
        g1 = blob1.color.g
        g2 = blob2.color.g
        b1 = blob1.color.b
        b2 = blob2.color.b

        # pylint: disable=line-too-long
        return math.sqrt(math.pow(r1-r2, 2)+math.pow(g1-g2, 2)+math.pow(b1-b2, 2))

    def add_new_feature(self, old_id, state, blob):
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
        # TODO(later): calculate this covariance
        covar = Matrix([[1, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0],
                        [0, 0, 1, 0, 0],
                        [0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 1]])
        self.potential_features[-new_id] = Feature(mean=mean, covar=covar)
        self.next_id += 1

    def cross_readings(self, old_reading, new_reading):
        '''
        Find the intersection of the two vectors defined by the position and
        direction of the two readings.
        Returns a tuple of the x, y pair where the two vectors intersect

        This will return not None even if the vectors/half-lines don't intersect

        See https://en.wikipedia.org/wiki/Line-line_intersection

        Input:
            (Odometry, Blob,) old_reading
            (Odometry, Blob,) new_reading
        Output:
            (float, float)
        '''

        x1 = old_reading[0].pose.pose.position.x
        y1 = old_reading[0].pose.pose.position.y
        h1 = quaternion_to_heading(old_reading[0].pose.pose.orientation)
        h1 = h1+old_reading[1].bearing
        x2 = x1+cos(h1)
        y2 = y1+sin(h1)

        x3 = new_reading[0].pose.pose.position.x
        y3 = new_reading[0].pose.pose.position.y
        h3 = quaternion_to_heading(new_reading[0].pose.pose.orientation)
        h3 = h3+new_reading[1].bearing
        x4 = x3+cos(h3)
        y4 = y3+sin(h3)

        ### temps

        t0 = x1*y2-y1*x2
        t1 = x3-x4
        t2 = x1-x2
        t3 = x3*y4-x4*y3
        t4 = t2
        t5 = y3-y4
        t6 = y1-y2
        t7 = t1

        t8 = t0
        t9 = t5
        t10 = t6
        t11 = t3
        t12 = t4
        t13 = t5
        t14 = t6
        t15 = t7

        ### end temps

        if (t4*t5-t6*t7) == 0 or (t12*t13-t14*t15) == 0:
            return None

        return ((t0*t1-t2*t3)/(t4*t5-t6*t7), (t8*t9-t10*t11)/(t12*t13-t14*t15),)

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
        current particle state and the given feature's mean

        bigH is the Jacobian of the measurement model with respect to the best
        estimate of the robot's position at the time of the update (computed at
        the mean of the feature, aka the predicted mean) PR p. 207

        Input:
            int feature_id
        Output:
            np.ndarray 4x3 (bigH)

        ...

        measurement = { bearing, r, g, b } = [x , y, heading (theta)]
        feature = { x, y, r, g, b} = [x, y, r, g, b]

        Jacobian =
        [[d bear/d x, d bear/d y, d bear/d r, d bear/d g, d bear/d b],
         [d r/d x,    d r/d y,    d r/d r,    d r/d g,    d r/d b],
         [d g/d x,    d g/d y,    d g/d r,    d g/d g,    d g/d b],
         [d b/d x,    d b/d y,    d b/d r,    d b/d g,    d b/d b]]

         Moving jacobian from derivative of measurement wrt pose
         to derivative of measurement wrt to feature state
        '''
        state = self.state
        mean_x = state.pose.pose.position.x
        mean_y = state.pose.pose.position.y
        feature_mean = self.get_feature_by_id(feature_id).mean
        feature_x = feature_mean[0]
        feature_y = feature_mean[1]

        # phi = atan2(dy, dx) - heading
        # q = (feature_x - mean_x)^2 + (feature_y - mean_y)^2
        q = float(pow(feature_x - mean_x, 2) + pow(feature_y - mean_y, 2))

        # d_phi/d_x = (feature_y - mean_y) / q
        try:
            d_bear_d_x = float(feature_y - mean_y) / q
        except ZeroDivisionError:
            d_bear_d_x = 0.0

        # d_phi/d_y = (feature_x - mean_x) / q
        try:
            d_bear_d_y = float(feature_x - mean_x) / q
        except ZeroDivisionError:
            d_bear_d_y = 0.0

        return Matrix([[d_bear_d_x, d_bear_d_y, 0.0, 0.0, 0.0],
                         [0.0,      0.0,        1.0, 0.0, 0.0],
                         [0.0,      0.0,        0.0, 1.0, 0.0],
                         [0.0,      0.0,        0.0, 0.0, 1.0]])

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
        
        other = mm(mm(bigH, old_covar), bigH.T)
        compilation = madd(other, Qt)
        return Matrix(compilation)

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
        return Matrix(mm(mm(old_covar, bigH.T), Qinv))

    def importance_factor(self, bigQ, blob, pseudoblob):
        '''
        Calculate the relative importance of this measurement (weight) to be
        used as part of resampling
        Input:
            np.ndarray bigQ (measurement covariance)
            Blob blob (recieved measurement)
            Blob pseudoblob (estimated measurement)
        '''
        v1 = 2.0*math.pi *magnitude(bigQ)
        v1 = pow(v1, -0.5)
        delz = blob_to_matrix(blob) - blob_to_matrix(pseudoblob)
        delzt = delz.T
        v2 = math.exp(-0.5 * mm(mm(delzt, inverse(bigQ)), delz))
        return v1 * v2

    def no_match_weight(self):
        '''
        return the default weight for when a particle doesn't match an
        observation to an existing feature
        '''
        # TODO(later): choose/tune the right arbitrary weight
        return 0.1

    def generate_measurement(self, featureid):
        '''
        Generate an expected measurement for the given featureid
        '''
        state = self.state
        s_x = state.pose.pose.position.x
        s_y = state.pose.pose.position.y
        feature = self.get_feature_by_id(featureid)
        f_x = feature.mean[0]
        f_y = feature.mean[1]

        bobby = Blob()
        bobby.bearing = math.atan2(f_y-s_y, f_x-s_x)
        # bobby.size = 1/math.sqrt(math.pow(f_x-s_x, 2)+math.pow(f_y-s_y, 2))
        bobby.color.r = feature.mean[2]
        bobby.color.g = feature.mean[3]
        bobby.color.b = feature.mean[4]

        return bobby

    # end FilterParticle

class Feature(object):
    def __init__(self, mean=None, covar=None):
        self.__immutable__=False
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
        if self.__immutable__:
            return None
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
        if self.__immutable__:
            return None
        adjust = msubtract(self.identity, mm(kalman_gain, bigH))
        self.covar = mm(adjust, self.covar)
        self.update_count += 1

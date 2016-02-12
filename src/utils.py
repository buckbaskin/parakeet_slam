import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from tf import transformations as tft

def quaternion_to_heading(quaternion):
    """
    Converts a quaternion to equivalent Euler yaw/heading
    input: nav_msgs.msg.Quaternion
    output: euler heading in radians
    """
    try:
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    except AttributeError:
        quat = quaternion
    yaw = tft.euler_from_quaternion(quat)[2]
    return yaw

def heading_to_quaternion(heading):
    """
    Converts a Euler yaw/heading angle to equivalent quaternion
    input: euler heading in radians
    output: nav_msgs.msg.Quaternion
    """

    quat = tft.quaternion_from_euler(0, 0, heading)

    quaternion = Quaternion()
    quaternion.x = quat[0]
    quaternion.y = quat[1]
    quaternion.z = quat[2]
    quaternion.w = quat[3]
    return quaternion

def dot_product(vec1, vec2):
    """
    calcuates the dot product of two tuples/vectors a, b
    input: 2 three-tuples a, vec2
    output: double: dot product
    """
    return vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2]

def cross_product(vec1, vec2):
    """
    calcuates the cross product of two tuples/vectors a, b
    input: 2 three-tuples vec1, vec2
    output: three-tuple (cross product of a, vec2)

     i    j    k
    a[0] a[1] a[2]
    b[0] b[1] b[2]
    """
    i = vec1[1]*vec2[2]-vec1[2]*vec2[1]
    j = vec1[0]*vec2[2]-vec1[2]*vec2[0]
    k = vec1[0]*vec2[1]-vec1[1]*vec2[0]
    return (i, j, k,)

def scale(vector, magnitude):
    """
    scales the given vector by the magnitude (scalar multiplication)
    input: three-tuple vector, double magnitude
    output: three-tuple scaled vector
    """
    return (vector[0]*magnitude, vector[1]*magnitude, vector[2]*magnitude)

def unit(vector):
    """
    returns the unit vector in the same direction as the given vector
    """
    length = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]+
        vector[2]*vector[2])
    if under_minimum(vector):
        raise ZeroDivisionError('vector length 0 cannot be scaled to a unit vector')
    return scale(vector, 1.0/length)

def under_minimum(vector):
    length = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]+
        vector[2]*vector[2])
    return length < .0001

def calc_errors(location, goal):
    """
    calculate errors in "x", "y", "theta" between a location and a goal

    input: two nav_msgs.msg.Odometry, a current best estimate of location
     and the goal
    output: a three-tuple representing error along the goal heading vector,
     error normal to that vector, and heading error

    example usage:
    odom = Odometry(current location)
    goal = Odometry(target location)
    along_axis, off_axis, heading = calc_errors(odom, goal)
    """
    along = along_axis_error(location, goal)
    off = off_axis_error(location, goal)
    heading = heading_error(location, goal)
    return (along, off, heading,)

def along_axis_error(location, goal):
    """
    calc error along the axis defined by the goal position and direction

    input: two nav_msgs.msg.Odometry, current best location estimate + goal
    output: double distance along the axis

    axis is defined by a vector from the unit circle aligned with the goal
     heading
    relative position is the vector from the goal x, y to the location x, y

    distance is defined by the dot product

    example use:
    see calc_errors above
    """
    relative_position_x = (location.pose.pose.position.x -
        goal.pose.pose.position.x)
    relative_position_y = (location.pose.pose.position.y -
        goal.pose.pose.position.y)

    # relative position of the best estimate position and the goal
    # vector points from the goal to the location
    relative_position = (relative_position_x, relative_position_y, 0.0)

    goal_heading = quaternion_to_heading(goal.pose.pose.orientation)
    goal_vector_x = math.cos(goal_heading)
    goal_vector_y = math.sin(goal_heading)

    # vector in the direction of the goal heading, axis of desired motion
    goal_vector = (goal_vector_x, goal_vector_y, 0.0)

    return dot_product(relative_position, goal_vector)

def off_axis_error(location, goal):
    """
    calc error normal to axis defined by the goal position and direction

    input: two nav_msgs.msg.Odometry, current best location estimate and
     goal
    output: double distance along the axis

    axis is defined by a vector from the unit circle aligned with the goal
     heading
    relative position is the vector from the goal x, y to the location x, y

    distance is defined by subtracting the parallel vector from the total
     relative position vector

    example use:
    see calc_errors above
    """
    # TODO(buckbaskin): assign a sign convention to off axis value

    relative_position_x = (location.pose.pose.position.x -
        goal.pose.pose.position.x)
    relative_position_y = (location.pose.pose.position.y -
        goal.pose.pose.position.y)
    relative_position_z = (location.pose.pose.position.z -
        goal.pose.pose.position.z)

    # relative position of the best estimate position and the goal
    # vector points from the goal to the location
    relative_position = (relative_position_x, relative_position_y,
        relative_position_z)

    if under_minimum(relative_position):
        return 0.0

    goal_heading = quaternion_to_heading(goal.pose.pose.orientation)
    goal_vector_x = math.cos(goal_heading)
    goal_vector_y = math.sin(goal_heading)

    # vector in the direction of the goal heading, axis of desired motion
    goal_vector = (goal_vector_x, goal_vector_y, 0.0)

    relative_along_goal = scale(unit(goal_vector),
        dot_product(relative_position, goal_vector))

    relative_normal_x = relative_position[0]-relative_along_goal[0]
    relative_normal_y = relative_position[1]-relative_along_goal[1]
    relative_normal_z = relative_position[2]-relative_along_goal[2]

    sign = 1.0

    crs_product = cross_product(goal_vector, relative_position)
    if crs_product[2] < 0:
        sign = -1.0

    return sign*math.sqrt(relative_normal_x*relative_normal_x+
        relative_normal_y*relative_normal_y+
        relative_normal_z*relative_normal_z)


def heading_error(location, goal):
    """
    return difference in heading between location and goal
    """
    loc_head = quaternion_to_heading(location.pose.pose.orientation)
    goal_head = quaternion_to_heading(goal.pose.pose.orientation)
    return loc_head - goal_head

def dist(odom1, odom2):
    """
    returns linear distance between two odometry messages
    """
    # pylint: disable=invalid-name
    # x and y accurately represent the axis that I'm referring to
    x = odom1.pose.pose.position.x - odom2.pose.pose.position.x
    y = odom1.pose.pose.position.y - odom2.pose.pose.position.y

    return math.sqrt(x*x+y*y)

def easy_Odom(x, y, heading=0.0, v=0.0, w=0.0, frame='odom'):
    odom = Odometry()
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation = heading_to_quaternion(heading)
    odom.twist.twist.linear.x = v
    odom.twist.twist.angular.z = w
    odom.header.frame_id = frame
    return odom

def minimize_angle(delta):
    while (delta > 2.0*math.pi):
        delta = delta - 2.0*math.pi

    while (delta < -2.0*math.pi):
        delta = delta + 2.0*math.pi
    if delta > math.pi:
        delta = -2.0*math.pi + delta
    if delta < -math.pi:
        delta = 2.0*math.pi + delta

    return delta

def is_close(a, b, sigma=4):
    if not (abs(a-b) < math.pow(1, -sigma)):
        print(a, 'not close to', b)
    return abs(a-b) < math.pow(1, -sigma)

def drange(start, stop, step):
    counter = start
    while counter < stop:
        yield counter
        counter += step

def version(version):
    def wrapper_function(function):
        function.version = version
        return function
    return wrapper_function
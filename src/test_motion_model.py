import copy as copy_module

from geometry_msgs.msg import Twist
from math import sin, cos
from numpy.random import normal
from prkt_core_v2 import FilterParticle
from utils import quaternion_to_heading, heading_to_quaternion


def motion_model(particle, twist, dt):
    # pS      h1        |
    # 0-----------------0
    # |                 h2
    v = twist.linear.x
    w = twist.angular.z

    # new_particle = FilterParticle()
    
    new_particle = copy_module.deepcopy(particle)
    print(new_particle.state.pose.pose.position.y)
    print(particle.state.pose.pose.position.y)

    dheading = twist.angular.z * dt

    drive_noise = normal(0, abs(.005*v)+abs(.001*w)+.0001, 1)
    print('original    '+str(twist.linear.x * dt))
    print('drive noise '+str(drive_noise))
    ds = twist.linear.x * dt + drive_noise

    print('and now, ds '+str(ds))

    prev_heading = quaternion_to_heading(particle.state.pose.pose.orientation)

    print('prev_heading '+str(prev_heading))

    heading_noise = normal(0, abs(.005*w)+abs(.001*v)+.0001, 1)
    heading_1 = prev_heading+dheading/2+heading_noise
    print('heading_1 '+str(heading_1))
    print('heading noise '+str(heading_noise))

    heading_noise = normal(0, abs(.005*w)+abs(.001*v)+.0001, 1)
    heading_2 = heading_1+dheading/2+heading_noise

    print('heading_2 '+str(heading_1))
    print('heading noise '+str(heading_noise))

    dx = ds*cos(heading_1)
    dy = ds*sin(heading_1)

    new_particle.state.pose.pose.position.x += dx
    new_particle.state.pose.pose.position.y += dy
    # pylint: disable=line-too-long
    new_particle.state.pose.pose.orientation = heading_to_quaternion(heading_2)

    return new_particle

if __name__ == '__main__':
    fpold = FilterParticle()

    fpold.state.pose.pose.position.y = 2.0

    twist = Twist()
    twist.linear.x = 1
    dt = .1

    dy_expected = twist.linear.x * dt

    fpnew = motion_model(fpold, twist, dt)

    dy_measured = fpnew.state.pose.pose.position.y - fpold.state.pose.pose.position.y

    print(dy_measured)
    print(dy_expected)
    print(dy_expected == dy_measured)
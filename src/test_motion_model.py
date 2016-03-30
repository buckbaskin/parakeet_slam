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

    new_particle = FilterParticle()
    
    new_particle = copy_module.deepcopy(particle)

    dheading = twist.angular.z * dt

    drive_noise = normal(0, abs(.005*v)+abs(.001*w)+.0001, 1)
    ds = twist.linear.x * dt + drive_noise

    prev_heading = quaternion_to_heading(particle.state.pose.pose.orientation)

    heading_noise = normal(0, abs(.005*w)+abs(.001*v)+.0001, 1)
    heading_1 = prev_heading+dheading/2+heading_noise

    heading_noise = normal(0, abs(.005*w)+abs(.001*v)+.0001, 1)
    heading_2 = heading_1+dheading/2+heading_noise

    print('asdf;kjasdf; '+str(heading_1 ))

    dx = ds*cos(heading_1)
    dy = ds*sin(heading_1)

    print('ds delta: '+str(ds - v*dt))

    new_particle.state.pose.pose.position.x += dx
    new_particle.state.pose.pose.position.y += dy
    # pylint: disable=line-too-long
    new_particle.state.pose.pose.orientation = heading_to_quaternion(heading_2)

    return new_particle

if __name__ == '__main__':
    fpold = FilterParticle()
    twist = Twist()
    twist.linear.x = 1
    dt = .1

    dx_expected = twist.linear.x * dt

    fpnew = motion_model(fpold, twist, dt)

    dx_measured = fpnew.state.pose.pose.position.x - fpold.state.pose.pose.position.x

    print(dx_measured)
    print(dx_expected)
    print(dx_expected == dx_measured)
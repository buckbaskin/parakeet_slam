#!/usr/bin/env python

'''
Drive a simple open loop set to test localization
'''

import rospy

from geometry_msgs.msg import Twist

if __name__ == '__main__':
	rospy.init_node('simple_drive')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	rate = rospy.Rate(2)

	while not rospy.is_shutdown():
		t = Twist()
		t.linear.x = 0.10
		t.angular.z = -0.10/3.0
		rospy.loginfo('pub pub pub')
		pub.publish(t)
		rospy.loginfo('pub pub')
		rate.sleep()
		rospy.loginfo('pub')

	rospy.loginfo('exited loop')

#!/usr/bin/env python

import rospy

'''
Follow the difference between two readings, a truth and a estimation reading
'''

last_truth = None
last_estimated = None

def truth(msg):
    last_truth = msg

def estimated(msg):
    last_estimated = msg

if __name__ == '__main__':
    rospy.init_node('')

    truth_sub = rospy.Subscriber('/odom', VizScan, truth)
    estimated_sub = rospy.Subscriber('/estimated', VizScan, truth)

    x_squared = 0.0
    y_squared = 0.0
    count = 0

    while not rospy.is_shutdown():
        if last_truth is not None and last_estimated is not None:
            count += 1
            x_squared += math.pow(last_truth.pose.pose.position.x - last_estimated.pose.pose.position.x, 2)
            y_squared += math.pow(last_truth.pose.pose.position.y - last_estimated.pose.pose.position.y, 2)
            last_truth = None
            last_estimated = None

    rospy.loginfo('avg x: %f and avg y: %f' % (math.sqrt(x_squared), math.sqrt(y_squared),))
#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

import sys

import numpy as np


last_odom = Odometry()
states = []


def odom_callback(msg):
    last_odom = msg


def rviz_callback(msg):
    global states, last_odom
    state = [last_odom.pose.pose.position.x, last_odom.pose.pose.position.y, msg.point.x, msg.point.y, \
        last_odom.twist.twist.linear.x, last_odom.twist.twist.angular.z]
    states.append(state)


def main():
    rospy.init_node('recorder')
    rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, odom_callback)
    rospy.Subscriber("/clicked_point", PointStamped, rviz_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
    np.save(f'/home/yigit/projects/is/data/data_raw/{sys.argv[1]}_{sys.argv[2]}_states.npy', np.asarray(states))
    print('fin')

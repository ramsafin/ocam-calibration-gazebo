#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState
from geometry_msgs.msg import Pose, Twist, Quaternion, Point
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


# subscribe to images and when 'S' is pressed save the image

def change_link_state(t, quat):
    try:
        f = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

        state = LinkState()
        state.link_name = 'chessboard::chessboard'
        state.pose.position = Point(t[0], t[1], t[2])
        state.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        status = f(state)
        print(status)

    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)


if __name__ == '__main__':
    rospy.init_node('ocam_calibration', anonymous=True)

    try:
        rospy.wait_for_service('/gazebo/set_link_state', 10)
    except rospy.ROSException as e:
        print("Error %s" % e)

    rpy = np.deg2rad([0, 0, 0])
    quat = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    print(quat)

    change_link_state([0.25, 1, 1], quat)

    rospy.spin()

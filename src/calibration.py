#!/usr/bin/env python

import sys
import Queue

import cv2
import numpy as np

import rospy
import rospkg
from cv_bridge import CvBridge
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState
from geometry_msgs.msg import Quaternion, Point
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler

bridge = CvBridge()
img_queue = Queue.Queue(25)

t_step = 0.005  # in meters
r_step = np.deg2rad(5)

# translation and rotation key mappings to the state changes
# for translation [x, y ,z] + [offset_x, offset_y, offset_z]
# for rotation [roll, pitch, yaw] + [offset_roll, offset_pitch, offset_yaw]
t_step_key_map = {'q': np.array([t_step, 0, 0]), 'a': np.array([-t_step, 0, 0]),
                  'w': np.array([0, t_step, 0]), 's': np.array([0, -t_step, 0]),
                  'e': np.array([0, 0, t_step]), 'd': np.array([0, 0, -t_step])}

r_step_key_map = {'r': np.array([r_step, 0, 0]), 'f': np.array([-r_step, 0, 0]),
                  't': np.array([0, r_step, 0]), 'g': np.array([0, -r_step, 0]),
                  'y': np.array([0, 0, r_step]), 'h': np.array([0, 0, -r_step])}


def clear_queue():
    for i in range(img_queue.qsize()):
        img_queue.get_nowait()


# subscribe to images and when 'S' is pressed save the image
def image_callback(image):
    global img_queue

    if img_queue.full():
        clear_queue()

    img_queue.put_nowait(bridge.imgmsg_to_cv2(image, "bgr8"))


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
        rospy.wait_for_service('/gazebo/set_link_state', 1)
    except rospy.ROSException as e:
        print("Error %s" % e)
        sys.exit(1)

    rospy.Subscriber('/setup/fisheye/image_raw', Image, image_callback, queue_size=10)

    img_counter = 0
    t = np.array([0.3, 0, 0.45])
    rpy = np.array(np.deg2rad([0, 0, 0]))

    change_link_state(t, quaternion_from_euler(*rpy))

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('ocam-calibration')

    # subscriber is working in another thread
    # here we use blocking stdin for chessboard pose control
    while not rospy.is_shutdown():
        key = sys.stdin.read(1)

        if key == '\n':
            continue  # skip 'Enter' key

        if key in t_step_key_map:
            t += t_step_key_map[key]
            change_link_state(t, quaternion_from_euler(*rpy))
        elif key in r_step_key_map:
            rpy += r_step_key_map[key]
            change_link_state(t, quaternion_from_euler(*rpy))
        elif key == 'x':
            filename = '{}/calibration/Fisheye_gazebo_{}.jpg'.format(pkg_path, img_counter)
            cv2.imwrite(filename, img_queue.get(), [int(cv2.IMWRITE_JPEG_QUALITY), 100])
            img_counter += 1
            print('Saved file: {}'.format(filename))
        elif key == 'z':  # reset roll-pitch-yaw
            rpy = np.array([0.0, 0.0, 0.0])
            change_link_state(t, quaternion_from_euler(*rpy))
        else:
            print ('Incorrect key is pressed: {}. Usage: qa/ws/ed/rf/tg/yh'.format(key))

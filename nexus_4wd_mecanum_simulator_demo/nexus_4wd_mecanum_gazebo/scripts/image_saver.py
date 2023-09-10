#! /usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

waiting = False
new_msg = False
msg = None
image_counter = -1
image_path = '/home/johnerzz/catkin_ws/src/nexus_4wd_mecanum_simulator_demo/nexus_4wd_mecanum_gazebo/scripts/images'


def camera_callback(data):
    global waiting, new_msg, msg, image_counter, image_path
    # the callback function will run everytime data is received but
    # it will do nothing unless the waiting variable is false
    # the waiting variable is controlled in main
    if not waiting:
        new_msg = True
        image_counter += 1
        msg = data
        cv_bridge = CvBridge()
        image_from_robot = cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8")
        rospy.loginfo('showing an image from camera 2.....')
        cv2.imshow('feed from camera 2', image_from_robot)
        cv2.imwrite( os.path.join(image_path , 'left_image' + str(image_counter) + '.jpg'), image_from_robot)
        cv2.waitKey(10) 
        # wait for 0.01 seconds. 
        # Without this short delay, cv2.imshow was not working. This delay does nothing else.

if __name__ == '__main__':
    rospy.init_node("image_saver")
    # define subscriber of topic '/depth_camera_2/color/image_raw'
    rospy.Subscriber('/depth_camera_2/color/image_raw', Image, camera_callback)
    r = rospy.Rate(1.5)    #subscriber runs at 0.5 Hz
    while not rospy.is_shutdown():
        if new_msg:
            #set waiting to True
            waiting = True
            new_msg = False
            r.sleep()
            #set waiting to False
            waiting = False

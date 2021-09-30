#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

THRESHOLD_DARK_GRAY = 120

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, 
  queue_size=1)

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

last_meas = 400

def image_callback(img_msg):

    # rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    # show_image(cv_image)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    
    row_dark_gray = 0
    col_dark_gray = 0
    count_dark_gray = 0
    skip = 7

    for i, row in enumerate(gray[-200:-1:skip]):
        for j, pixel in enumerate(row[::skip]):
            if pixel < THRESHOLD_DARK_GRAY:
                row_dark_gray += 200 + i*skip
                col_dark_gray += j*skip
                count_dark_gray += 1
    
    if count_dark_gray == 0:
        show_image(cv_image)
        return
    row_com = row_dark_gray // count_dark_gray
    col_com = col_dark_gray // count_dark_gray



    global last_meas
    last_meas = col_com

    move = Twist()
    # move.linear.x = 0.5
    p = -(col_com - len(gray[0]) / 2.0)
    d = 0#(col_com - last_meas)
    pd = 3 * (p + d) / len(gray[0])
    move.angular.z = pd
    move.linear.x = 2-abs(pd)

    pub.publish(move)
    
    rospy.loginfo('loginfo: ')
    rospy.loginfo(col_com)
    rospy.loginfo(len(gray[0]))
    rospy.loginfo(p)
    rospy.loginfo(d)
    rospy.loginfo(pd)

    out_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

    # out_frame[row_com][col_com] = [255, 0, 0]
    cv2.circle(out_frame, (col_com, len(gray) - row_com), 30, (255, 0, 0), -1)
    show_image(out_frame)

sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,image_callback)

rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.5
move.angular.z = 0.5

while not rospy.is_shutdown():
    # pub.publish(move)
    rate.sleep()

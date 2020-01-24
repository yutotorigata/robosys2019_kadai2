#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from operator import itemgetter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class colorimage(object):
  def __init__(self):
    self._image_pub = rospy.Publisher("original", Image, queue_size = 1)
    self._blue_pub   = rospy.Publisher('blue', Image, queue_size=1)
    self._yellow_pub = rospy.Publisher('yellow', Image, queue_size=1)
    self._red_pub    = rospy.Publisher('red', Image, queue_size=1)

# camera image receive
    self._image_sub  = rospy.Subscriber('/usb_cam/image_raw',     Image, self.callback)
    self._bridge     = CvBridge()

    self.flag = 1
    self.kernel = np.ones((5, 5), np.uint8)
    self.f = 698.568629
    self.theta = math.pi / 6
    self.height = 300

  def callback(self, data):
#camera import
    cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
    image = cv_image

#convert to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#go to mask function!!
    try:
      mask_blue, mask_yellow, mask_red = self.mask(hsv)

    except CvBridgeError, e:
      print e

  def mask(self, HSV):
    low_blue = np.array([94,75,22])
    up_blue = np.array([130, 255, 255])

    low_yellow = np.array([10, 91, 0])
    up_yellow = np.array([30, 255, 255])

    low_red1 = np.array([161, 113, 0])
    up_red1 = np.array([179, 255, 255])
    low_red2 = np.array([0, 77, 45])
    up_red2 = np.array([11, 255, 255])

    mask_b = cv2.inRange(HSV, low_blue, up_blue)
    mask_y = cv2.inRange(HSV, low_yellow, up_yellow)
    mask_r1 = cv2.inRange(HSV, low_red1, up_red2)
    mask_r2 = cv2.inRange(HSV, low_red2, up_red2)
    mask_r = mask_r1 + mask_r2

    return mask_b, mask_y, mask_r

if __name__ == "__main__":
  rospy.init_node("color_image")
  color = colorimage()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    pass

#!/usr/bin/env/python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist

def callback(message):
  rospy.loginfo("aa %s",message.data)

rospy.init_node('list_sub')
sub=rospy.Subscriber('list_data',Twist,callback)
rospy.spin()

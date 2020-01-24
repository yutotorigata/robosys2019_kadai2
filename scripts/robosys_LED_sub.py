#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist

#GPIO pin number 
GPIO.setmode(RPi.GPIO.BCM)
#number 18 output
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)
def callback(led):
    color = msg.linear.x

    if (color == 1.0):    #blue
        for x in xrange(2):
            GPIO.output(2, True)
            time.sleep(2)
            GPIO.output(2, False)
            time.sleep(2)
        GPIO.cleanup()
    elif (color == 2.0):    #yellow
        for x in xrange(2):
            GPIO.output(2, True)
            GPIO.output(3, True)
            time.sleep(2)
            GPIO.output(2, False)
            GPIO.output(3, False)
            time.sleep(2)
        GPIO.cleanup()
    elif (color == 3.0):    #red
        for x in xrange(2):
            GPIO.output(2, True)
            GPIO.output(3, True)
            GPIO.output(4, True)
            time.sleep(2)
            GPIO.output(2, False)
            GPIO.output(3, False)
            GPIO.output(4, False)
            time.sleep(2)
        GPIO.cleanup()

rospy.init_node('listener')
LED_sub = rospy.Subscriber('/LED_color',Twist,callback)
rospy.spin()

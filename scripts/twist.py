#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy

class colorimage(object):
    def __init__(self):
        self._t_pub = rospy.Publisher("/t", Twist, queue_size=1)
        while(True):
            self.callback()

    def callback(self):
        number = Twist()
        print("kuma death!")
        number.linear.x = 1
        print("sachi = hentai")
        self._t_pub.publish(number)
        print("sachi")


if __name__ == "__main__":
    rospy.init_node("color_image")
    color = colorimage()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


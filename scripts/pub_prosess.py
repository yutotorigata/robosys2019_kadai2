#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

class prosess(object):
    def __init__(self):
        self.prosess1 = rospy.Publisher("/prosess", Int32, queue_size=1)

        #while(1):
        self.prosess()

    def prosedd(self):
        prosess_1 = Int32()
        prosess_1.data = 1
        self.prosess1.publish(prosess_1)

if __name__ == "__main__":
    rospy.init_node("pro")
    pro = prosess()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

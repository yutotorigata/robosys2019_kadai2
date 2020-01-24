#!/usr/bin/env python

import rospy
from chinocon.image_converter import Image_converter

def main():
    rospy.init_node("cv_cam_test")
    converter = Image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
if __name__ == "__main__":
    main()

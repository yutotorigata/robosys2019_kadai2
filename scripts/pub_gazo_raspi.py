#!/usr/bin/env python

import rospy
import cv2
import math
import numpy as np
from operator import itemgetter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

flag   = 1
kernel = np.ones((5,5), np.uint8)
f      = 475.542113
theta  = math.pi/6
height = 290

class colorimage(object):
    def __init__(self):
        self._image_pub  = rospy.Publisher('original', Image, queue_size=1)
        self._blue_pub   = rospy.Publisher('blue', Image, queue_size=1)
        self._yellow_pub = rospy.Publisher('yellow', Image, queue_size=1)
        self._red_pub    = rospy.Publisher('red', Image, queue_size=1)
        # camera image receive
        self._image_sub  = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self._bridge     = CvBridge()

        self._number_pub = rospy.Publisher("/number", Twist, queue_size=1)
        
    def callback(self, data):
        #camera import
        cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        image    = cv_image

        # convert to hsv
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #go to mask function!!
        try:
            mask_b, mask_y, mask_r    = self.mask(hsv)
            image_b, image_y, image_r = self.bitwise(image, mask_b, mask_y, mask_r)
        except CvBridgeError as e:
            print('ee')

        try:
            #go to def contour()!!
            blue_list, blue_color, blue_kukei = self.contours(image_b, image, "blue")
            blue_list = blue_list[:blue_color]
            blue_list = blue_list.tolist()
            blue_list.sort(key = itemgetter(2))

            yellow_list, yellow_color, yellow_kukei = self.contours(image_y, image, "yellow")
            yellow_list = yellow_list[:yellow_color]
            yellow_list = yellow_list.tolist()
            yellow_list.sort(key = itemgetter(2))
           
            red_list, red_color, red_kukei = self.contours(image_r, image, "red")
            red_list = red_list[:red_color]
            red_list = red_list.tolist()
            red_list.sort(key = itemgetter(2))
            
            color_list = [[100000000,1000000000,100000000,10000000,1000000000]]
            color_list.extend(blue_list)
            color_list.extend(yellow_list)
            color_list.extend(red_list)
            color_list.sort(key=itemgetter(2))
            color      = color_list[0:1]
            xt_list    = []
            yt_list    = []
            ht_list    = []
            theta_list = []
            c_list     = []
            for x in color:
                xt_list.append(x[0])
                for y in color:
                    yt_list.append(y[1])
                    for z in color:
                        ht_list.append(z[2])
                        for xx in color:
                            theta_list.append(xx[3])
                            for yy in color:
                                c_list.append(yy[4])
            b_l = 1.0
            y_l = 2.0
            r_l = 3.0

        except CvBridgeError as e:
            print('eee')

        try:

            xt    = color_list[0][0]
            yt    = color_list[0][1]
            ht    = color_list[0][2]
            theta = color_list[0][3]
            Color = color_list[0][4]

            #print(c_list)
            #print("ht=")
            #print(ht-200)
            #print("theta=")
            #print(theta)
            print("color=")
            print(Color)
            print("\n")

        except CvBridgeError as e:
            print("eeeeeeeeeeeeeeeee")

        # opencv program to ros program
        try:
            self._image_pub.publish(self._bridge.cv2_to_imgmsg(image, "bgr8"))
            self._blue_pub.publish(self._bridge.cv2_to_imgmsg(image_b, "mono8"))
            self._yellow_pub.publish(self._bridge.cv2_to_imgmsg(image_y, "mono8"))
            self._red_pub.publish(self._bridge.cv2_to_imgmsg(image_r, "mono8"))

        except CvBridgeError as e:
            print('eeee')

        try:
            number_l = Twist()
            #number_l.linear.x  = ht-200
            #number_l.linear.y  = theta
            number_l.linear.z  = Color
            self._number_pub.publish(number_l)
        except:
            pass
    
    def mask(self, HSV):
        #hsv value set
        #blue
        low_blue   = np.array([94,75,22])
        up_blue    = np.array([130,255,255])
        #yellow
        low_yellow = np.array([10,91,0])
        up_yellow  = np.array([30,255,255])
        #red
        low_red1   = np.array([161,113,0])
        up_red1    = np.array([179,255,255])
        low_red2   = np.array([0,77,45])
        up_red2    = np.array([11,255,255])

        #convert to mask image
        #blue
        mask_blue   = cv2.inRange(HSV, low_blue, up_blue)
        #yellow   
        mask_yellow = cv2.inRange(HSV, low_yellow, up_yellow)
        #red
        mask_red1   = cv2.inRange(HSV, low_red1, up_red1)
        mask_red2   = cv2.inRange(HSV, low_red2, up_red2)
        mask_red    = mask_red1 + mask_red2
        return mask_blue, mask_yellow, mask_red

    def bitwise(self, IMAGE, mask_blue, mask_yellow, mask_red):
        #mask image add original image
        bw_blue   = cv2.bitwise_and(IMAGE, IMAGE, mask=mask_blue)
        bw_yellow = cv2.bitwise_and(IMAGE, IMAGE, mask=mask_yellow)
        bw_red    = cv2.bitwise_and(IMAGE, IMAGE, mask=mask_red)
        #noise procesing
        bw_blue   = self.noise(bw_blue)
        bw_yellow = self.noise(bw_yellow)
        bw_red    = self.noise(bw_red)
        return bw_blue, bw_yellow, bw_red
        
    def noise(self, bw):
        #noise smoothing
        clear_img = cv2.GaussianBlur(bw,(33,33),1)
        #RGB to GRAY
        gray  = cv2.cvtColor(clear_img,cv2.COLOR_BGR2GRAY)
        gray2 = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        gray3 = cv2.morphologyEx(gray2, cv2.MORPH_CLOSE, kernel)
        gray4 = cv2.GaussianBlur(gray3,(33,33),1) 
        return gray4

    def color_ball(self, img, x, y, w, h):
        cx_color = (x-320)+w/2
        cy_color = (-1)*((y-240)+h/2)
        delta_theta_color = math.atan2(cy_color,f)
        yt_color = (height-(h/2))/math.tan(theta-delta_theta_color)
        f2_color = math.sqrt(f**2+cy_color**2)
        xt_color = (cx_color*math.sqrt(yt_color**2+height**2))/f2_color
        ht_color = math.sqrt(xt_color**2+yt_color**2)
        theta_rad_color = (math.pi/2)-math.atan2(yt_color, xt_color)
        theta_color = math.degrees(theta_rad_color)
        return xt_color, yt_color,  ht_color, theta_color

    def contours(self, IMG, img, COLOR):
        ball_imgEdge, contours, ball_hierarchy = cv2.findContours(IMG, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        color_list = np.zeros((5,5))
        kukei_list = np.zeros((5,4))
        color = 0
        k     = 0

        if len(contours) != 0:
            for k in range(len(contours)):
                cnt  = contours[k]
                M    = cv2.moments(cnt)
                area = cv2.contourArea(cnt)
                peri = cv2.arcLength(cnt, True)

                if peri > 0.0 and area > 200:
                    enkeido = 4*math.pi*area/(peri*peri)
                else:
                    enkeido = 0.0

                if enkeido > 0.8:
                    x, y, w, h = cv2.boundingRect(cnt)
                    area2      = cv2.contourArea(cnt)
                    k          += 1

                    xt_c, yt_c, ht_c, theta_c = self.color_ball(img, x, y, w, h)
                    color_list[color][0] = xt_c
                    color_list[color][1] = yt_c
                    color_list[color][2] = ht_c
                    color_list[color][3] = theta_c

                    kukei_list[color][0] = x
                    kukei_list[color][1] = y
                    kukei_list[color][2] = w
                    kukei_list[color][3] = h

                    if(COLOR == "blue"):
                        color_list[color][4] = 1
                        color_list_blue = color_list.tolist()
                    elif(COLOR == "yellow"):
                        color_list[color][4] = 2
                        color_list_yellow = color_list.tolist()
                    elif(COLOR == "red"):
                        color_list[color][4] = 3
                        color_list_red = color_list.tolist()

                    color += 1

        return color_list, color, kukei_list

if __name__ == "__main__":
    rospy.init_node("color_image")
    color = colorimage()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

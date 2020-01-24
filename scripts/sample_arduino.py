#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import Float32
from sensor_mags.msg import Image
rospy.init_node('talker')
pub = rospy.Publisher('led',Float32,queue_size=1)

#numbers are in millimeters
flag = 1 #pogram start
cap = cv2.VideoCapture(0)
kernel = np.ones((5,5), np.uint8)
f = 698.568629   #focal length
theta = math.pi/6       #camera angle
height = 300     #camera height
cap.set(cv2.CAP_PROP_FPS,60)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

class ColorExtract(object):
  def __init__(self):
    self._vel_pub = rospy.Publisher('cmd_vel', Float32,queue_size=10)
    self._blue_pub = rospy.Publisher('blue_image', Image, queue_size=1)
    self._yellow_pub = rospy.Publisher('yellow_image', Image, queue_size=1)
    self._red_pub = rospy.Publisher('red_image', Image, queue_size=1) 
    self._image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
    self._vel =Float32()

  def main():
    #camera import 
    ret,frame = cap.read()
    
    #RGB space convert HSV space & mask data in mask function 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_blue, mask_yellow, mask_red = mask(hsv, frame)

    #each mask data and original data in contours function
    mask_blue, FRAME2 = contours(mask_blue, frame, "blue")
    mask_yellow, FRAME2 = contours(mask_yellow, frame, "yellow")
    mask_red, FRAME2 = contours(mask_red, frame, "red")

    #create center line and center point of camera image 
    height = FRAME2.shape[0]
    width = FRAME2.shape[1]
    FRAME2 = cv2.line(FRAME2, (0,height/2),(width-1,height/2),(0,255,0),1)
    FRAME2 = cv2.line(FRAME2, (width/2,0),(width/2,height-1),(0,255,0),1)

    #each image show
    cv2.imshow("blue",mask_blue)
    cv2.imshow("yellow",mask_yellow)
    cv2.imshow("red",mask_red)
    cv2.imshow("color",FRAME2)

  def mask(HSV,FRAME):    #mask function
    #blue HSV value setting
    lower_blue = np.array([100,124,16])
    upper_blue = np.array([130,255,255])
    #create mask image & original image and mask image intergration
    img_mask_blue = cv2.inRange(HSV, lower_blue, upper_blue)
    img_color_blue = cv2.bitwise_and(FRAME, FRAME, mask=img_mask_blue)
    img_color_blue = noise(img_color_blue)  #img_color_blue data in noise function

    #yellow HSV value setting
    lower_yellow = np.array([10,108,95])
    upper_yellow = np.array([50,255,255])
    #create mask image & original image and mask image intergration
    img_mask_yellow = cv2.inRange(HSV, lower_yellow, upper_yellow)
    img_color_yellow = cv2.bitwise_and(FRAME, FRAME, mask=img_mask_yellow)
    img_color_yellow = noise(img_color_yellow)  #img_color_yellow data in noise function

    #red HSV value setting
    lower_red1 = np.array([165,139,0])
    upper_red1 = np.array([179,255,255])
    lower_red2 = np.array([0,120,70])
    upper_red2 = np.array([10,255,255])
    #create mask image & original image and mask image intergration
    img_mask_red1 = cv2.inRange(HSV, lower_red1, upper_red1)
    img_mask_red2 = cv2.inRange(HSV, lower_red2, upper_red2)
    img_mask_red = img_mask_red1 + img_mask_red2 
    img_color_red = cv2.bitwise_and(FRAME, FRAME, mask=img_mask_red)  
    img_color_red = noise(img_color_red)  #img_color_red data in noise function

    return img_color_blue, img_color_yellow, img_color_red

  def noise(img_color):   #noise function
    #noise smoothing of img_color data
    img_color = cv2.GaussianBlur(img_color,(33,33),1)
    #RGB space convert GRAY space
    gray = cv2.cvtColor(img_color,cv2.COLOR_BGR2GRAY)
    gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE,kernel)
    gray = cv2.GaussianBlur(gray, (33,33),1)  
    return gray

  def contours(MASK, FRAME2, COLOR):  #contour function
    ball_imgEdge, contours, ball_hierarchy = cv2.findContours(MASK,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
      for k in range(len(contours)):
        cnt = contours[k]
        M = cv2.moments(cnt)
        area = cv2.contourArea(cnt) #menseki
        perimeter = cv2.arcLength(cnt,True) #shui-cho
   
        if perimeter > 0.0 and area > 200:
          enkeido = 4*math.pi*area/(perimeter*perimeter)  #enkeido  calculation
        else:
          enkeido = 0.0
      
        if enkeido > 0.8:  
          x,y,w,h = cv2.boundingRect(cnt)
          area2 = cv2.contourArea(cnt)
          if (COLOR == "blue"):#blue    
           FRAME2 = cv2.rectangle(FRAME2,(x,y),(x+w,y+h),(255,0,0),4)
           FRAME2 = cv2.circle(FRAME2,(x+w/2,y+h/2),1,(255,0,0),3)
           cx_blue = (x-320)+w/2  #distance between camera and ball__x coordinate
           cy_blue =(-1)*((y-240)+h/2)  #distance between camera and ball__y coordinate
            
           while True:
            #coordinate conversion start
            delta_theta_blue = math.atan2(cy_blue,f)
            yt_blue = (height-(h/2))/math.tan(theta-delta_theta_blue)  #ball position__y coordinate
            f2_blue = math.sqrt(f**2+cy_blue**2)
            xt_blue = (cx_blue*math.sqrt(yt_blue**2+height**2))/f2_blue  #ball_position__x coordinate
            ht_blue = math.sqrt(xt_blue**2+yt_blue**2)  #ball_position__hypotenuse coodinate
            theta_rad_blue = (math.pi/2)-math.atan2(yt_blue,xt_blue)
            theta_blue = math.degrees(theta_rad_blue)

            #decimal point specification
            xt_blue = round(xt_blue,3)
            yt_blue = round(yt_blue,3)
            ht_blue = round(ht_blue,3)
            theta_blue = round(theta_blue,3)

            #data text start in image
            k+=1
            font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
            cv2.putText(FRAME2,"BLUE%d"%k,(x,y-100),font,0.8,(255,0,0),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"xt="+str(xt_blue)+"mm",(x,y-75),font,0.8,(255,0,0),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"yt="+str(yt_blue)+"mm",(x,y-55),font,0.8,(255,0,0),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"Lt="+str(ht_blue)+"mm",(x,y-35),font,0.8,(255,0,0),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"theta="+str(theta_blue)+"deg",(x,y-15),font,0.8,(255,0,0),1,cv2.LINE_AA)

            print("BLUE%d"%k)
            print("cx_blue=" + str(cx_blue) + " cy_blue=" + str(cy_blue))
            print("xt_blue=" + str(xt_blue) + " yt_blue=" + str(yt_blue) + " Lt_blue=" + str(ht_blue) + " theta_blue=" + str(theta_blue))
            break

          elif (COLOR == "yellow"):#yellow
           FRAME2 = cv2.rectangle(FRAME2,(x,y),(x+w,y+h),(0,241,255),4)
           FRAME2 = cv2.circle(FRAME2,(x+w/2,y+h/2),1,(0,241,255),3)
           cx_yellow = (x-320)+w/2
           cy_yellow = (-1)*((y-240)+h/2)

           while True:
            #coordinate conversion start
            delta_theta_yellow = math.atan2(cy_yellow,f)
            yt_yellow = (height-(h/2))/math.tan(theta-delta_theta_yellow)  #ball position__y coordinate
            f2_yellow = math.sqrt(f**2+cy_yellow**2)
            xt_yellow = (cx_yellow*math.sqrt(yt_yellow**2+height**2))/f2_yellow  #ball_position__x coordinate
            ht_yellow = math.sqrt(xt_yellow**2+yt_yellow**2) #ball_position__hypotenuse coodinate
            theta_rad_yellow = (math.pi/2)-math.atan2(yt_yellow,xt_yellow)
            theta_yellow = math.degrees(theta_rad_yellow)

            #decimal point specification
            xt_yellow = round(xt_yellow,3)
            yt_yellow = round(yt_yellow,3)
            ht_yellow = round(ht_yellow,3)
            theta_yellow = round(theta_yellow,3)

            #data text start in image
            k+=1
            font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
            cv2.putText(FRAME2,"YELLOW%d"%k,(x,y-100),font,0.8,(0,241,255),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"xt="+str(xt_yellow)+"mm",(x,y-75),font,0.8,(0,241,255),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"yt="+str(yt_yellow)+"mm",(x,y-55),font,0.8,(0,241,255),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"Lt="+str(ht_yellow)+"mm",(x,y-35),font,0.8,(0,241,255),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"theta="+str(theta_yellow)+"deg",(x,y-15),font,0.8,(0,241,255),1,cv2.LINE_AA)

            print("YELLOW%d"%k)
            print ("cx_yellow=" + str(cx_yellow) + " cy_yellow=" + str(cy_yellow))
            print ("xt_yellow=" + str(xt_yellow) + " yt_yellow=" + str(yt_yellow) + " Lt_yellow=" + str(ht_yellow) + " theta_yellow=" + str(theta_yellow))
            break

          elif (COLOR == "red"):#red
           FRAME2 = cv2.rectangle(FRAME2,(x,y),(x+w,y+h),(0,0,255),4)
           FRAME2 = cv2.circle(FRAME2,(x+w/2,y+h/2),1,(0,0,255),3)
           cx_red = (x-320)+w/2
           cy_red = (-1)*((y-240)+h/2)

           while True:
            #coordinate conversion start
            delta_theta_red = math.atan2(cy_red,f)
            yt_red = (height-(h/2))/math.tan(theta-delta_theta_red)  #ball position__y coordinate
            f2_red = math.sqrt(f**2+cy_red**2)
            xt_red = (cx_red*math.sqrt(yt_red**2+height**2))/f2_red  #ball_position__x coordinate
            ht_red = math.sqrt(xt_red**2+yt_red**2)  #ball_position__hypotenuse coodinate
            theta_rad_red = (math.pi/2)-math.atan2(yt_red,xt_red)
            theta_red = math.degrees(theta_rad_red)

            #decimal point specification
            xt_red = round(xt_red,3)
            yt_red = round(yt_red,3)
            ht_red = round(ht_red,3)
            theta_red = round(theta_red,3)

            #data text start in image
            k+=1
            font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
            cv2.putText(FRAME2,"RED%d"%k,(x,y-100),font,0.8,(0,0,255),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"xt="+str(xt_red)+"mm",(x,y-75),font,0.8,(0,0,255),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"yt="+str(yt_red)+"mm",(x,y-55),font,0.8,(0,0,255),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"Lt="+str(ht_red)+"mm",(x,y-35),font,0.8,(0,0,255),1,cv2.LINE_AA)
            cv2.putText(FRAME2,"theta="+str(theta_red)+"deg",(x,y-15),font,0.8,(0,0,255),1,cv2.LINE_AA)

            print("RED%d"%k)
            print ("cx_red=" + str(cx_red) + " cy_red=" + str(cy_red))
            print ("xt_red=" + str(xt_red) + " yt_red=" + str(yt_red) + " Lt_red=" + str(ht_red) + " theta_red=" + str(theta_red))
            break

    return  MASK, FRAME2            


if __name__ == "__main__":  #first
  while(flag == 1):       
    main()
    k = cv2.waitKey(1)
    if k == ord("q"):
      break
  cap.release()
  cv2.destroyAllWindows()

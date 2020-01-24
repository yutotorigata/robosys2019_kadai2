#!/usr/bin/python
#-*- coding: utf-8 -*-

import cv2
import numpy as np
import math
from operator import itemgetter

#numbers are in millimeters
flag   = 1                                            #pogram start
cap    = cv2.VideoCapture(0)
kernel = np.ones((5,5), np.uint8)
f      = 698.568629                                   #focal length
theta  = math.pi/6                                    #camera angle
height = 300                                          #camera height

cap.set(cv2.CAP_PROP_FPS,60)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

def main():
  #camera import 
  ret,frame = cap.read()
  
  #RGB space convert HSV space & mask data in mask function 
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  mask_blue, mask_yellow, mask_red = mask(hsv, frame) #go to def mask():!!!!!!!!   return image
  
  #each mask data and original data in contours function
  blue_list, b_color, blue_kukei = contours(mask_blue, frame, "blue") #go to def contour():!!!!!!
  blue_list = blue_list[:b_color]
  blue_list = blue_list.tolist()
  #print(blue_list)
  blue_list.sort(key=itemgetter(0))

  yellow_list, y_color, yellow_kukei = contours(mask_yellow, frame, "yellow")
  yellow_list = yellow_list[:y_color]
  yellow_list = yellow_list.tolist()
  yellow_list.sort(key=itemgetter(0))
  #print(yellow_list)

  red_list, r_color, red_kukei = contours(mask_red, frame, "red") 
  red_list = red_list[:r_color]
  red_list = red_list.tolist()
  red_list.sort(key=itemgetter(0))
  #print(red_list)

  color_list = [[100000,100000,1000000,10000000000]]
  color_list.extend(blue_list)
  color_list.extend(yellow_list)
  color_list.extend(red_list)
  color_list.sort(key=itemgetter(0))
  #print("color=\n"+str(color_list)+"\n")
  color = color_list[0:1]
  print("color_list=")
  print(color)
  ht_list = []
  theta_list = []
  c_list = []
  for x in color:
     ht_list.append(x[0])
     for y in color:
       theta_list.append(y[1])
       for z in color:
         c_list.append(z[3])

  b_l = 1.0
  y_l = 2.0
  r_l = 3.0

  print("ht_list=")
  print(ht_list)
  print("theta_list=")
  print(theta_list)
  if b_l in c_list:
    print("blue_list=")
  elif y_l in c_list:
    print("yellow_list=")
  elif r_l in c_list:
    print("red_list=")
  print(c_list)
  print("ht=")
  print(color_list[0][0])
  print("theta=")
  print(color_list[0][1])
  print("color=")
  print(color_list[0][3])
  print("\n")

  #each image show
  cv2.imshow("blue", mask_blue)
  cv2.imshow("yellow", mask_yellow)
  cv2.imshow("red", mask_red)
  cv2.imshow('ball', frame)

#mask function
def mask(HSV,FRAME):
  #blue HSV value setting
  lower_blue = np.array([94,75,22])
  upper_blue = np.array([130,255,255])

  #create mask image & original image and mask image intergration
  img_mask_blue  = cv2.inRange(HSV, lower_blue, upper_blue)
  img_color_blue = cv2.bitwise_and(FRAME, FRAME, mask = img_mask_blue)
  #img_color_blue data in noise function
  img_color_blue = noise(img_color_blue)   #go to def noise():!!!!!!!! 

  #yellow HSV value setting
  lower_yellow = np.array([10,91,0])
  upper_yellow = np.array([30,255,255])
  #create mask image & original image and mask image intergration
  img_mask_yellow  = cv2.inRange(HSV, lower_yellow, upper_yellow)
  img_color_yellow = cv2.bitwise_and(FRAME, FRAME, mask = img_mask_yellow)
  #img_color_yellow data in noise function
  img_color_yellow = noise(img_color_yellow)  

  #red HSV value setting
  lower_red1 = np.array([161,113,0])
  upper_red1 = np.array([179,255,255])
  lower_red2 = np.array([0,77,45])
  upper_red2 = np.array([11,255,255])
  #create mask image & original image and mask image intergration
  img_mask_red1 = cv2.inRange(HSV, lower_red1, upper_red1)
  img_mask_red2 = cv2.inRange(HSV, lower_red2, upper_red2)
  img_mask_red  = img_mask_red1 + img_mask_red2 
  img_color_red = cv2.bitwise_and(FRAME, FRAME, mask = img_mask_red)  
  #img_color_red data in noise function
  img_color_red = noise(img_color_red)  

  return img_color_blue, img_color_yellow, img_color_red

#noise function
def noise(img_color):   
  #noise smoothing of img_color data
  img_color = cv2.GaussianBlur(img_color, (33,33), 1)
  #RGB space convert GRAY space
  gray      = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
  gray      = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
  gray      = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
  gray      = cv2.GaussianBlur(gray, (33,33), 1)  
  return gray

def color_ball(FRAME2,x,y,w,h):                         #Value calculation
  cx_color   = (x-320)+w/2                              #distance between camera and ball__x coordinate
  cy_color   = (-1)*((y-240)+h/2)                       #distance between camera and ball__y coordinate
 
  #coordinate conversion start
  delta_theta_color = math.atan2(cy_color,f)
  yt_color = (height-(h/2))/math.tan(theta-delta_theta_color)         #ball position__y coordinate
  f2_color = math.sqrt(f**2+cy_color**2)
  xt_color = (cx_color*math.sqrt(yt_color**2+height**2))/f2_color       #ball_position__x coordinate
  ht_color = math.sqrt(xt_color**2+yt_color**2)                        #ball_position__hypotenuse coodinate
  theta_rad_color   = (math.pi/2)-math.atan2(yt_color, xt_color)
  theta_color       = math.degrees(theta_rad_color)
  return ht_color, theta_color
      
#contours function
def contours(MASK,FRAME2,COLOR):  
  ball_imgEdge, contours, ball_hierarchy = cv2.findContours(MASK, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  color_list = np.zeros((5, 4))         
  kukei_list = np.zeros((5, 4))
  color = 0
  k     = 0

  if len(contours) != 0:
    for k in range(len(contours)):
      cnt  = contours[k]
      M    = cv2.moments(cnt)
      area = cv2.contourArea(cnt)                           #menseki
      peri = cv2.arcLength(cnt, True)                       #shui-cho
 
      if peri > 0.0 and area > 200:
        enkeido = 4*math.pi*area/(peri*peri)                #enkeido  calculation
      else:
        enkeido = 0.0
    
      if enkeido > 0.8:
         x,y,w,h = cv2.boundingRect(cnt)
         area2   = cv2.contourArea(cnt)
         k       += 1

         ht_c, theta_c = color_ball(FRAME2,x,y,w,h)  #go to def color_ball():!!!!
         color_list[color][0] = ht_c
         color_list[color][1] = theta_c
         color_list[color][2] = k

         kukei_list[color][0] = x
         kukei_list[color][1] = y
         kukei_list[color][2] = w
         kukei_list[color][3] = h
        

         if (COLOR == "blue"):
           color_list[color][3] = 1
           color_list_b = color_list.tolist()

         elif (COLOR == "yellow"):
           color_list[color][3] = 2
           color_list_y = color_list.tolist()

         elif (COLOR == "red"):
           color_list[color][3] = 3
           color_list_r = color_list.tolist()

         color += 1 
  
  return color_list, color, kukei_list            

if __name__ == "__main__":  #first
  while(flag == 1):       
    main()
    k = cv2.waitKey(1)
    if k == ord("q"):
      break
  cap.release()
  cv2.destroyAllWindows()

'''
Author: Xu.WANG
Date: 2021-10-29 19:59:27
LastEditTime: 2021-10-29 21:33:32
LastEditors: Xu.WANG
Description: 
FilePath: \Kiri2D\scripts\test\circle_painter_v1.py
'''
import numpy as np
import cv2

height = 1080
width = 1920


img = np.ones((height,width,3), np.uint8) * 255

radius = 500
center_x = width/2
center_y = height/2

def r2d(r):
    return (r*180/np.pi + 360) % 360

def demo1():
    for i in range(int(center_x-radius),int(center_x+radius)):
        for j in range(int(center_y - np.sqrt(radius**2 - (i-center_x)**2)), int(center_y + np.sqrt(radius**2 - (i-center_x)**2))):
            
            # default color
            img[j][i] = (0,0,255)

            base_line_vx = 0
            base_line_vy = radius
            cur_line_vx = i - center_x
            cur_line_vy = j - center_y

            dot= base_line_vx*cur_line_vx +base_line_vy* cur_line_vy
            det = base_line_vx * cur_line_vy - cur_line_vx * base_line_vy

            radian = np.arctan2(det,dot)
            degree = r2d(radian)

            if(degree>0 and degree < 90):
                img[j][i] = (255,0,0)
            elif(degree>90 and degree < 180):
                img[j][i] = (255,0,255)
            elif(degree>180 and degree < 270):
                img[j][i] = (255,255,0)
            else:
                img[j][i] = (0,255,255)

def demo2():
    for i in range(int(center_x-radius),int(center_x+radius)):
        for j in range(int(center_y - np.sqrt(radius**2 - (i-center_x)**2)), int(center_y + np.sqrt(radius**2 - (i-center_x)**2))):
            
            # default color
            img[j][i] = (0,0,255)

            base_line_vx = 0
            base_line_vy = radius
            cur_line_vx = i - center_x
            cur_line_vy = j - center_y

            dot= base_line_vx*cur_line_vx +base_line_vy* cur_line_vy
            det = base_line_vx * cur_line_vy - cur_line_vx * base_line_vy

            radian = np.arctan2(det,dot)
            degree = r2d(radian)

            if(degree>45 and degree < 135):
                img[j][i] = (255,0,0)
            elif(degree>135 and degree < 225):
                img[j][i] = (255,0,255)
            elif(degree>225 and degree < 315):
                img[j][i] = (255,255,0)
            else:
                img[j][i] = (0,255,255)

demo1()
cv2.imwrite('./demo1.png', img)

demo2()
cv2.imwrite('./demo2.png', img)
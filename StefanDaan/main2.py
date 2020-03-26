#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 19 06:15:25 2020

@author: daan
"""

# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import cv2
from matplotlib import pyplot as plt
import glob
from EstimateLinearFlowField import estimate_linear_flow_field
import time
import pandas

t = time.time()
# do stuff


def rescale_frame(frame, percent=25):
    width = int(np.shape(frame)[1] * percent/ 100)
    height = int(np.shape(frame)[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

####
#load images
####
    
#filenames = [img for img in glob.glob("/home/daan/Documents/AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646/*.jpg")]

filenames = [img for img in glob.glob("/home/daan/Documents/AE4317_2019_datasets/cyberzoo_poles_panels/20190121-140205/*.jpg")]

filenames.sort() # ADD THIS LINE

images = []
for img in filenames:
    n= cv2.imread(img)
    images.append(n)    
    
#only select images of collision to test algorithm
images = images
for i in range(len(images)):
    images[i] = images[i][125:375]


#reduce resolution of images to speed up process
red_images = []
for i in images:
    temp = rescale_frame(i)
    red_images.append(temp)

#define grid with x and y value for each point
images = red_images 
points_old = np.zeros([np.shape(red_images[0])[0] * np.shape(red_images[0])[1], 2])
for i in np.arange(0, np.shape(red_images[1])[0], 1):
    for j in np.arange(0, np.shape(red_images[1])[1], 1):
        points_old[(i) * np.shape(red_images[0])[1] + j] = [j,i]
        
#write video from images
height,width,layers = images[0].shape
size = (width,height)
out = cv2.VideoWriter('project2.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

for i in range(len(images)):
    out.write(images[i])
out.release()

####
#Perform dense optical flow calculations
####

cap = cv2.VideoCapture('project2.avi')

#define feature vector
ret, frame1 = cap.read()
prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255

# iterate over the images:
n_images = len(images);
FoE_over_time = np.zeros([n_images, 2]);
horizontal_motion_over_time = np.zeros([n_images, 1]);
vertical_motion_over_time = np.zeros([n_images, 1]);
divergence_over_time = np.zeros([n_images, 1]);
errors_over_time = np.zeros([n_images, 1]);
elapsed_times = np.zeros([n_images,1]);
ttc_over_time = np.zeros([n_images,1]);
time_to_contact = 0.0;
im = 0;
while(1):
    ret, frame2 = cap.read()
    next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 10, 7, 1.2, 0)
    mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
    hsv[...,0] = ang*180/np.pi/2
    hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
    rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    
    flow = np.reshape(flow,[np.shape(red_images[0])[0] * np.shape(red_images[0])[1], 2])
    pu, pv, err = estimate_linear_flow_field(points_old, flow, RANSAC=True, n_iterations=100, error_threshold=10.0)
        
    divergence = (pu[0] + pv[1]) / 2.0;
    small_threshold = 1E-5;   
    if(abs(divergence) > small_threshold):
        time_to_contact = 1 / divergence;
    
        
    # book keeping:
    ttc_over_time[im] = time_to_contact * (1/30);
    
    prvs = next
    im = im + 1
    if im == (len(images) - 1):
        break;
    
cap.release()
cv2.destroyAllWindows()


#clean data
for i in range(len(ttc_over_time)):
    if abs(ttc_over_time[i]) > 10:
        ttc_over_time[i] = ttc_over_time[i-1]
        

ttc_smoothed = pandas.ewma(abs(ttc_over_time), com = 0.99)

plt.plot(abs(ttc_smoothed))



#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 13:38:15 2020

@author: max
"""

import numpy as np
import cv2
from matplotlib import pyplot as plt
import glob
from EstimateLinearFlowField import estimate_linear_flow_field
import time


filenames = [img for img in glob.glob("/home/max/Downloads/AE4317_2019_datasets/cyberzoo_poles/20190121-135009/*.jpg")]

filenames.sort() 

def rescale_frame(frame, percent=25):
    width = int(np.shape(frame)[1] * percent/ 100)
    height = int(np.shape(frame)[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)


images = []
for img in filenames:
    n= cv2.imread(img)
    images.append(n)
    
red_images = []
for i in images:
    temp = rescale_frame(i)
    red_images.append(temp)

show = [90, 390]

height,width,layers = images[0].shape
size = (width,height)
out = cv2.VideoWriter('project2.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

for i in range(len(images)):
    out.write(images[i])
out.release()

for i in show:
    cv2.imshow('frame' + str(i), images[i])    

cv2.waitKey(0)
cv2.destroyAllWindows()
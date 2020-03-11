#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 08:03:08 2020

@author: daan
"""
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import re
import time
import sys

def estimate_linear_flow_field(points_old, flow_vectors, RANSAC=True, n_iterations=100, error_threshold=10.0):
    
    n_points = points_old.shape[0];
    sample_size = 3; # minimal sample size is 3
    
    if(n_points >= sample_size):
        
        if(not RANSAC):
            
            # *****************************************
            # TODO: investigate this estimation method:
            # *****************************************
            
            # estimate a linear flow field for horizontal and vertical flow separately:
            # make a big matrix A with elements [x,y,1]
            A = np.concatenate((points_old, np.ones([points_old.shape[0], 1])), axis=1);
            
            # Moore-Penrose pseudo-inverse:
            # https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
            pseudo_inverse_A = np.linalg.pinv(A);
            
            # target = horizontal flow:
            u_vector = flow_vectors[:,0];
            # solve the linear system:
            pu = np.dot(pseudo_inverse_A, u_vector);
            # calculate how good the fit is:
            errs_u = np.abs(np.dot(A, pu) - u_vector);
            
            # target = vertical flow:
            v_vector = flow_vectors[:,1];
            pv = np.dot(pseudo_inverse_A, v_vector);
            errs_v = np.abs(np.dot(A, pv) - v_vector);
            err = (np.mean(errs_u) + np.mean(errs_v)) / 2.0;
            
        else:
            # This is a RANSAC method to better deal with outliers
            # matrices and vectors for the big system:
            A = np.concatenate((points_old, np.ones([points_old.shape[0], 1])), axis=1);
            u_vector = flow_vectors[:,0];
            v_vector = flow_vectors[:,1];
            
            # solve many small systems, calculating the errors:
            errors = np.zeros([n_iterations, 2]);
            pu = np.zeros([n_iterations, 3])
            pv = np.zeros([n_iterations, 3])
            for it in range(n_iterations):
                inds = np.random.choice(range(n_points), size=sample_size, replace=False);
                AA = np.concatenate((points_old[inds,:], np.ones([sample_size, 1])), axis=1);
                pseudo_inverse_AA = np.linalg.pinv(AA);
                # horizontal flow:
                u_vector_small = flow_vectors[inds, 0];
                # pu[it, :] = np.linalg.solve(AA, UU);
                pu[it,:] = np.dot(pseudo_inverse_AA, u_vector_small);
                errs = np.abs(np.dot(A, pu[it,:]) - u_vector);
                errs[errs > error_threshold] = error_threshold;
                errors[it, 0] = np.mean(errs);
                # vertical flow:
                v_vector_small = flow_vectors[inds, 1];
                # pv[it, :] = np.linalg.solve(AA, VV);
                pv[it, :] = np.dot(pseudo_inverse_AA, v_vector_small);
                errs = np.abs(np.dot(A, pv[it,:]) - v_vector);
                errs[errs > error_threshold] = error_threshold;
                errors[it, 1] = np.mean(errs);
            
            # take the minimal error
            errors = np.mean(errors, axis=1);
            ind = np.argmin(errors);
            err = errors[ind];
            pu = pu[ind, :];
            pv = pv[ind, :];
    else:
        # not enough samples to make a linear fit:
        pu = np.asarray([0.0]*3);
        pv = np.asarray([0.0]*3);
        err = error_threshold;
        
    return pu, pv, err;
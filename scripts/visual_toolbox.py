#!/usr/bin/env pyhon
import cv2
import numpy as np

def shade_grey_est(grayimg):
    size = grayimg.size
    power = np.power(np.float32(grayimg), 6)
    normalized_p_norm = np.power(np.sum(power)/size, 1/6.0)
    return normalized_p_norm

def shade_grey(img):
    """Minkowski P-Norm Shades of Grey"""
    b,g,r = cv2.split(img)
    illumination_est = np.mean([shade_grey_est(x) for x in [b,g,r]])
    b_corrected = illumination_est/np.mean(b)*b
    b_corrected = b_corrected.clip(max=240)
    g_corrected = illumination_est/np.mean(g)*g
    g_corrected = g_corrected.clip(max=240)
    r_corrected = illumination_est/np.mean(r)*r
    r_corrected = r_corrected.clip(max=240)
    return cv2.merge((np.uint8(b_corrected),np.uint8(g_corrected),np.uint8(r_corrected)))

def single_deflicker(grayimgs):
    logimgs = [cv2.log(np.float32(x)) for x in grayimgs]
    median = np.median(logimgs, axis=0)
    diff = np.abs(logimgs[-1] - median)
    blur = cv2.GaussianBlur(diff, (3,3), 1, 1)
    illumination_est = np.exp(blur)
    output = grayimgs[-1]/(illumination_est)
    return output

def motionDeflicker(imgs):
    """A motion compensated approach to remove sunlight flicker
    Choice of low-pass filter could be changed or used a different standard deviation
    """
    b = [x[:,:,0] for x in imgs] 
    g = [x[:,:,1] for x in imgs] 
    r = [x[:,:,2] for x in imgs] 
    b_corrected = single_deflicker(b)
    g_corrected = single_deflicker(g)
    r_corrected = single_deflicker(r)
    return cv2.merge((np.uint8(b_corrected),np.uint8(g_corrected),np.uint8(r_corrected)))

def single_homomorphic_filter(grayimg):
    log =  np.nan_to_num(np.log(np.float32(grayimg)))
    blur = np.nan_to_num(cv2.GaussianBlur(log, (21,21), 21))
    output = np.exp(cv2.addWeighted(log, 0.5, blur, 0.5, 0))
    print(output)
    return output

def homomorphic_filter(img):
    """Homomorphic filtering"""
    b,g,r = cv2.split(img)
    b_corrected = single_homomorphic_filter(b)
    g_corrected = single_homomorphic_filter(g)
    r_corrected = single_homomorphic_filter(r)
    return cv2.merge((np.uint8(b_corrected),np.uint8(g_corrected),np.uint8(r_corrected)))

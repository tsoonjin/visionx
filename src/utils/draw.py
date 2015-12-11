#!/usr/bin/env python
"""Functions to annotate image and add visual information 
to image
"""
import cv2
import numpy as np

from output import Output

'''Color codes'''
BLUE = (255,128,0)
GREEN = (0,255,0)
RED = (0,0,255)
YELLOW = (0,255,255)
WHITE = (255,255,255)

CENTERCAM_COLOR = WHITE     #color of square at center of cam

def draw_square(canvas, center, color, offset=8, thickness=2):
    """Draws a square
    Args:
        offset: distance from center which determines size of square
    """
    top_left = (center[0] - offset, center[1] + offset)
    bot_right = (center[0] + offset, center[1] - offset)
    cv2.rectangle(canvas, top_left, bot_right, color, thickness)

def draw_circle(canvas, center, color, rad=4, thickness=2):
    cv2.circle(canvas, center, rad, color, thickness)

def draw_text(canvas, text, bot_left, color, size=0.6,
        fontface=cv2.FONT_HERSHEY_SIMPLEX):
    """Draws a text 
    Args:
        bot_left: position of bottom-leftmost text on canvas
    """
    cv2.putText(canvas, text, bot_left, fontface, size, color)

def ypos(starty, inc):
    while True:
        yield starty
        starty += inc

def draw_debugtext(canvas, output):
    startx, starty = 20, ypos(30,25)  #initializes starting coordinate of text column
    draw_text(canvas, "dx: {}, dy: {}".format(output.dx, output.dy), 
        (startx, next(starty)), BLUE)
    draw_text(canvas, "area_ratio: {}".format(output.area_ratio), 
        (startx, next(starty)), BLUE)

def draw_cross(canvas, center, color, size=(5,5), thickness=2):
    top_left = (center[0] - size[0], center[1] + size[1])
    top_right = (center[0] + size[0], center[1] + size[1])
    bot_left = (center[0] - size[0], center[1] - size[1])
    bot_right = (center[0] + size[0], center[1] - size[1])
    cv2.line(canvas, bot_left, top_right, color, thickness)
    cv2.line(canvas, bot_right, top_left, color, thickness)

def create_debugimg(output):
    """Generates image for debugging
    Args:
        output: Output object that stores details of detected object
    Returns:
        outimg: output image with added visual information
    """
    canvas = np.zeros_like(output.outimg)
    draw_square(canvas, output.center, CENTERCAM_COLOR, offset=10)  #draws center of screen
    if output.detected:
        draw_cross(canvas, output.centroid, RED)  #draws center of detected object
        draw_debugtext(canvas, output)
    return canvas

if __name__ == "__main__":
    output = Output(cv2.imread("../../test/resources/img/bin1.jpg"))
    output.centroid = (500,300)
    output.area_ratio = 50000
    '''
    blank = np.zeros_like(output.outimg)
    draw_cross(blank, output.center, RED)
    draw_square(blank, output.center, CENTERCAM_COLOR, offset=10)
    '''
    blank = create_debugimg(output)
    cv2.imshow("test", blank)
    cv2.waitKey(0)

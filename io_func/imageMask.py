import numpy as np
from PIL import Image
import cv2
import os

dir = r'C:\Users\jonat\OneDrive\Documents\Robot\Example0\model\testing\\'
for file in os.listdir(dir):
    print(file)
    filename = os.fsdecode(file)
    if filename.endswith( ('.jpeg', '.png', '.gif', '.jpg') ):
        # load image
        img = cv2.imread(dir + filename)
        #convert the BGR image to HSV colour space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #set the lower and upper bounds for the green hue
        lower_green = np.array([0,0,0])
        upper_green = np.array([255,255,150])
        #create a mask for green colour using inRange function
        mask = cv2.inRange(hsv, lower_green, upper_green)
        #perform bitwise and on the original image arrays using the mask
        res = cv2.bitwise_and(img, img, mask=mask)
        # Output
        cv2.imwrite(dir + filename, res)

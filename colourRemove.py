import cv2
import glob
import numpy as np
directory = './data/model/visualize/'
for filename in glob.glob(directory + '/*.jpg'):
    print(filename)
    # Load image
    im = cv2.imread(filename)
    y,x,_ = im.shape
    # Define lower and upper limits of our blue
    im[np.where((im>[200,200,200]).all(axis=2))] = [255,255,255]
    #im[np.where((im<[50,70,70]).all(axis=2))] = [255,255,255]
    #im[np.where((im>[140,140,140]).all(axis=2) & (im<[160,160,160]).all(axis=2))] = [255,255,255]
    im[:, 0:950] = (255,255,255)
    im[:, 4300:] = (255,255,255)
    im[2700:, :] = (255,255,255)

    cv2.imwrite(filename, im)

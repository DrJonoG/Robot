import numpy as np
import os
import cv2

def wMatrix(matrix, filename, path, abbrev=""):
    # Create path if not exists
    if not os.path.exists(path):
        os.makedirs(path)

    # Write matrix file
    mat = np.matrix(matrix)
    file = path + abbrev + filename + '.txt'
    with open(file,'wb') as f:
        for line in mat:
            np.savetxt(f, line, fmt='%.8f')

def undistort(source, destination, config):
    # Load files
    intrinsicMatrix = np.asmatrix(np.loadtxt(config['calibration']['working_dir'] + "\\axyb\\" + "intrinsic.txt"))
    optimalIntrinsicMatrix = np.asmatrix(np.loadtxt(config['calibration']['working_dir'] + "\\axyb\\" + "intrinsicOptimised.txt"))
    distCoeffs = np.asmatrix(np.loadtxt(config['calibration']['working_dir'] + "\\axyb\\" + "distCoeffs.txt"))
    for filename in os.listdir(destination):
        imgToAdapt = cv2.imread(destination + filename)
        imgToAdapt = cv2.undistort(imgToAdapt, intrinsicMatrix, distCoeffs, None, optimalIntrinsicMatrix)
        cv2.imwrite(destination + "\\" + filename, imgToAdapt)

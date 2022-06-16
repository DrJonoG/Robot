import numpy as np
import cv2, os
import glob
import copy
from shutil import copyfile

class calibrateCamera(object):
    def __init__(self, file_path, output_path, cb_w, cb_h, cb_size):
        self.image_source = file_path
        self.output_destination = output_path
        self.calibration_destination = ""
        self.cb_w = int(cb_w)
        self.cb_h = int(cb_h)
        self.cb_size = float(cb_size)
        self.images = glob.glob(self.image_source + '*.jpg') + glob.glob(self.image_source + '*.png') + glob.glob(self.image_source + '*.tif')

    def create_folders(self, paths):
        for path in paths:
            if not os.path.exists(path):
                os.makedirs(path)

    # Output path is the same as input, the output is a series of transformation matrices
    def calibrate(self):
        np.set_printoptions(precision=5, suppress=True)

        print("==> Performing camera calibration")

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.cb_h * self.cb_w, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cb_w, 0:self.cb_h].T.reshape(-1, 2)
        #objp *= self.cb_size

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        found, unfound = 0, 0

        image_count = len(self.images)

        if image_count <= 3:
            print("==> Calibration error: insufficient images. 3 required %s provided." % str(image_count))
            return

        for i in range(image_count):
            img = cv2.imread(self.images[i])
            # Adjust gamma - under some light conditions the checkerboard quality is reduced, increase/decreasing value can result in more stable camera estimates
            #img = adjust_gamma(img, 0.7)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            # If found, add object points, image points (after refining them)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.cb_w, self.cb_h), flags=cv2.ADAPTIVE_THRESH_GAUSSIAN_C)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp * float(self.cb_size))

                corners2 = cv2.cornerSubPix(gray,corners,(14,14),(-1,-1),criteria)
                imgpoints.append(corners2)

                found += 1
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (self.cb_w,self.cb_h), corners2,ret)
                #cv2.imshow('img',img)
                cv2.imwrite(self.output_destination + "/projected/" + str(i) + ".jpg", img)
            else:
                print("==> Error: Unable to detect checkerboard at " + self.images[i])
                unfound += 1

        print("==> Found %s checkerboards, didn't find %s checkerboards." % (found, unfound))

        # Extracting camera parameters
        retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, flags=cv2.CALIB_RATIONAL_MODEL)

        # Save intrinsic
        np.savetxt(self.output_destination + "/txt/intrinsic.txt", cameraMatrix,  fmt='%.6f')

        # Create all of the necessary folders
        self.create_folders([self.output_destination, self.output_destination + "/txt/", self.output_destination + "/images/"])

        # Export the camera matrices
        for i in range(0, len(self.images)):
            file_name = str(i).zfill(8)
            camera_matrix = np.append(cv2.Rodrigues(rvecs[i])[0],tvecs[i],axis=1)
            # Calculate projection matrix
            projection_matrix = np.dot(cameraMatrix, camera_matrix)
            projection_matrix = np.append(projection_matrix, [[0, 0, 0, 1]], axis=0)
            # Camera matrix
            camera_matrix = np.append(camera_matrix, [[0, 0, 0, 1]], axis=0)
            # Save files
            np.savetxt(self.output_destination + "/txt/" + file_name + ".txt", camera_matrix,  fmt='%.6f')
            np.savetxt(self.output_destination + "/projection/" + file_name + ".txt", projection_matrix,  fmt='%.6f')
            copyfile(self.images[i], self.output_destination + "/images/" + file_name + ".jpg")


    def adjust_gamma(self, image, gamma=1.0):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                          for i in np.arange(0, 256)]).astype("uint8")

        # apply gamma correction using the lookup table
        return cv2.LUT(image, table)

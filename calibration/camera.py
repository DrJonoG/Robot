from active import calibration

import numpy as np
import cv2, os
import glob
import copy
from shutil import copyfile
from PIL import Image, ImageDraw

class calibrateCamera(object):
    def __init__(self, working_dir, config):
        self.config = config
        # Main directory
        self.output_destination = working_dir
        # Sub directories
        self.image_destination = self.output_destination + "\\images\\"
        self.axyb_destination = self.output_destination + "\\axyb\\"
        self.projection_destination = self.output_destination + "\\projection\\"
        self.projected_destination = self.output_destination + "\\projected\\"
        # Checkerboard variables
        self.cb_w = int(self.config['calibration']['cb_width'])
        self.cb_h = int(self.config['calibration']['cb_length'])
        self.cb_size = float(self.config['calibration']['cb_size'])
        # Create necessary folders
        self.create_folders([self.image_destination, self.output_destination, self.axyb_destination, self.projection_destination, self.projected_destination])
        # custom intrinsic parameters
        self.customIntrin = np.eye(3, dtype=np.float64)
        self.customIntrin[0, 0] = 3079.39
        self.customIntrin[1, 1] = 3079.39
        self.customIntrin[0, 2] = 1920
        self.customIntrin[1, 2] = 1080


    def capture(self, robot, cam, turntable):
        # Active vision for the capturing of images
        calibration.capture(robot, cam, turntable, self.image_destination, self.axyb_destination, self.output_destination)

    def averageIntrinsics(self):
        source = self.output_destination + "\\turntable\\"
        # Check if folder exists
        if not os.path.exists(source):
            print("==> Error: Unable to average. No turntable calibration found.")
            return

        # Average intrinsics from turntable calibration
        intrinsic = np.empty([3,3])
        intrinsicOptimised = np.empty([3,3])
        distCoeffs = np.empty(12)
        counter = 0
        for filename in os.listdir(source):
            # update file count
            counter += 1
            # Load and add
            intrinsic = intrinsic + np.loadtxt(source + filename + '\\axyb\intrinsic.txt')
            intrinsicOptimised = intrinsicOptimised + np.loadtxt(source + filename + '\\axyb\intrinsicOptimised.txt')
            distCoeffs = distCoeffs + np.loadtxt(source + filename + '\\axyb\distCoeffs.txt')

        # Backup old file and save new
        intrinsic = intrinsic / counter
        os.rename(self.axyb_destination + "/intrinsic.txt", self.axyb_destination + "/intrinsic_bk.txt")
        np.savetxt(self.axyb_destination + "/intrinsic.txt", intrinsic,  fmt='%.8f')
        #
        intrinsicOptimised = intrinsicOptimised / counter
        os.rename(self.axyb_destination + "/intrinsicOptimised.txt", self.axyb_destination + "/intrinsicOptimised_bk.txt")
        np.savetxt(self.axyb_destination + "/intrinsicOptimised.txt", intrinsicOptimised,  fmt='%.8f')
        #
        distCoeffs = distCoeffs / counter
        os.rename(self.axyb_destination + "/distCoeffs.txt", self.axyb_destination + "/distCoeffs_bk.txt")
        np.savetxt(self.axyb_destination + "/distCoeffs.txt", distCoeffs,  fmt='%.8f')

    def create_folders(self, paths):
        for path in paths:
            if not os.path.exists(path):
                os.makedirs(path)

    # Output path is the same as input, the output is a series of transformation matrices
    def calibrate(self):
        # Update image count
        self.images = glob.glob(self.image_destination + '*.jpg') + glob.glob(self.image_destination + '*.png') + glob.glob(self.image_destination + '*.tif')

        # numpy output
        np.set_printoptions(precision=6, suppress=True)

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

        errorList = []
        for i in range(image_count):
            img = cv2.imread(self.images[i])
            # Adjust gamma - under some light conditions the checkerboard quality is reduced, increase/decreasing value can result in more stable camera estimates
            #img = adjust_gamma(img, 0.7)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # If found, add object points, image points (after refining them)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.cb_w, self.cb_h), flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp * float(self.cb_size))

                corners2 = cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
                imgpoints.append(corners2)
                found += 1

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (self.cb_w,self.cb_h), corners2,ret)

                # Write to file
                #cv2.imwrite(self.output_destination + "/projected/" + str(i) + ".jpg", img)
            else:
                print("==> Error: Unable to detect checkerboard at " + self.images[i])
                errorList.append(i)
                unfound += 1

        print("==> Found %s checkerboards, didn't find %s checkerboards." % (found, unfound))

        # Extracting camera parameters
        retval, intrinsicMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, flags=cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL)
        # For defining intrin
        #retval, intrinsicMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], self.customIntrin, None, flags=cv2.CALIB_FIX_PRINCIPAL_POINT + cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_FIX_FOCAL_LENGTH + cv2.CALIB_USE_INTRINSIC_GUESS )

        #
        sampleImage = cv2.imread(self.images[0])
        # Get the dimensions of the image
        height, width = sampleImage.shape[:2]
        # Refine camera matrix
        # Returns optimal camera matrix and a rectangular region of interest
        optimalIntrinsicMatrix, roi = cv2.getOptimalNewCameraMatrix(intrinsicMatrix, distCoeffs, (width,height), 1, (width,height))

        mean_error = 0
        # Estimate accuracy
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], intrinsicMatrix, distCoeffs)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print(f"==> Estimated error (pixels): {round(mean_error/len(objpoints),4)}")

        # Save intrinsic
        np.savetxt(self.axyb_destination + "/intrinsic.txt", intrinsicMatrix,  fmt='%.8f')
        np.savetxt(self.axyb_destination + "/intrinsicOptimised.txt", optimalIntrinsicMatrix,  fmt='%.8f')
        np.savetxt(self.axyb_destination + "/distCoeffs.txt", distCoeffs,  fmt='%.8f')

        counter = 0
        # Export the camera matrices
        for i in range(0, len(self.images)):
            # Check if checkerboard is visible
            if i in errorList:
                print("==> Skipping " + self.images[i] + " as checkerboard not visible.")
                continue

            file_name = str(i).zfill(8)

            # The rotation and translation of the camera
            A = np.append(cv2.Rodrigues(rvecs[counter])[0],tvecs[counter],axis=1)

            # If testing output projected
            if self.config['general']['testing'] == "True":
                # Load
                imgToAdapt = cv2.imread(self.images[i])
                # Undistort image
                undistorted_image = imgToAdapt#cv2.undistort(imgToAdapt, intrinsicMatrix, distCoeffs, None, optimalIntrinsicMatrix)
                # Save image
                cv2.imwrite(self.projected_destination + "\\" + file_name + ".jpg", undistorted_image)
                # Load image as PIL
                input_image = Image.open(self.projected_destination + "\\" + file_name + ".jpg")
                # Calculate projection matrix
                projection_matrix = np.dot(intrinsicMatrix, A)
                projection_matrix = np.append(projection_matrix, [[0, 0, 0, 1]], axis=0)
                point = [0,0,0,1]
                # Project point to camera
                v = (projection_matrix @ point) # dot product
                x = int(v[0] / v[2])
                y = int(v[1] / v[2])
                # Draw ellipse
                draw = ImageDraw.Draw(input_image)
                draw.ellipse((x-5, y-5, x+5, y+5), fill=(255,0,0,0))
                # Saving the final output
                input_image.save(self.projected_destination + "\\" + file_name + ".jpg")
                input_image.close()

            # Camera matrix
            A = np.append(A, [[0, 0, 0, 1]], axis=0)
            # Save files
            np.savetxt(self.axyb_destination + "/A" + file_name + ".txt", A,  fmt='%.8f')
            np.savetxt(self.projection_destination + file_name + ".txt", projection_matrix,  fmt='%.8f')

            counter = counter + 1



    def adjust_gamma(self, image, gamma=1.0):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                          for i in np.arange(0, 256)]).astype("uint8")

        # apply gamma correction using the lookup table
        return cv2.LUT(image, table)

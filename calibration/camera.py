from active import calibration
from calibration import functions as functions
import numpy as np
import cv2, os
import cv2.aruco as aruco
import glob
import copy
from shutil import copyfile
from PIL import Image, ImageDraw
import pathlib
from datetime import datetime
from pathlib import Path
import re
import shutil
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
        self.distort = self.config['calibration']['distort']
        # Create necessary folders
        self.create_folders([self.image_destination, self.output_destination, self.axyb_destination, self.projection_destination, self.projected_destination])
        # custom intrinsic parameters
        self.customIntrin = np.eye(3, dtype=np.float64)
        self.customIntrin[0, 0] = float(self.config['intrin']['focal_x'])
        self.customIntrin[1, 1] = float(self.config['intrin']['focal_y'])
        self.customIntrin[0, 2] = float(self.config['intrin']['center_x'])
        self.customIntrin[1, 2] = float(self.config['intrin']['center_y'])


    def capture(self, robot, cam, turntable):
        # Active vision for the capturing of images
        calibration.capture(robot, cam, turntable, self.image_destination, self.axyb_destination, self.output_destination, self.config['calibration']['positions'])

    def averageIntrinsics(self):
        source = self.output_destination + "\\turntable\\"
        # Check if folder exists
        if not os.path.exists(source):
            print("==> Error: Unable to average. No turntable calibration found.")
            return

        # Average intrinsics from turntable calibration
        intrinsic = np.empty([3,3])
        intrinsicOptimised = np.empty([3,3])
        distCoeffs = np.empty(5)
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
        #os.rename(self.axyb_destination + "/intrinsic.txt", self.axyb_destination + "/intrinsic_bk.txt")
        np.savetxt(self.axyb_destination + "/intrinsic_avg.txt", intrinsic,  fmt='%.8f')
        #
        intrinsicOptimised = intrinsicOptimised / counter
        #os.rename(self.axyb_destination + "/intrinsicOptimised.txt", self.axyb_destination + "/intrinsicOptimised_bk.txt")
        np.savetxt(self.axyb_destination + "/intrinsicOptimised_avg.txt", intrinsicOptimised,  fmt='%.8f')
        #
        distCoeffs = distCoeffs / counter
        #os.rename(self.axyb_destination + "/distCoeffs.txt", self.axyb_destination + "/distCoeffs_bk.txt")
        np.savetxt(self.axyb_destination + "/distCoeffs_avg.txt", distCoeffs,  fmt='%.8f')

    def create_folders(self, paths):
        for path in paths:
            if not os.path.exists(path):
                os.makedirs(path)


    def reorderFolders():
        pass

    # Output path is the same as input, the output is a series of transformation matrices
    def calibrate(self):
        # Update image count
        self.imageInitial = glob.glob(self.image_destination + '*.jpg') + glob.glob(self.image_destination + '*.png')
        self.images = []
        # numpy output
        np.set_printoptions(precision=6, suppress=True)
        print(datetime.now().strftime('%H:%M:%S') + " ==> Performing camera calibration")

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 0.0001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.cb_h * self.cb_w, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cb_w, 0:self.cb_h].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        # Number of detected checkerboards
        found, unfound = 0, 0

        # Get intiial starting images
        image_count = len(self.imageInitial)

        # Ensure at least three images
        if image_count <= 3:
            print(datetime.now().strftime('%H:%M:%S') + " ==> Calibration error: insufficient images. 3 required %s provided." % str(image_count))
            return

        # List of boards where not detected
        errorList = []
        # Loop initial image set
        for i in range(image_count):
            # Update consoles
            print(datetime.now().strftime('%H:%M:%S') + " ==> Detecting Checkerboards " + str(i) + " of " + str(image_count), end='\r')
            # Load image
            img = cv2.imread(self.imageInitial[i])
            # Adjust gamma
            # under some light conditions the checkerboard quality is reduced, increase/decreasing value can result in more stable camera estimates
            #img = adjust_gamma(img, 0.7)

            # Convert to gray
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # If found, add object points, image points (after refining them)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.cb_w, self.cb_h), flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp * float(self.cb_size))
                # Sub pixel corners
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)
                found += 1

                # Draw and display the corners
                if self.config['general']['testing'] == "True":
                    img = cv2.drawChessboardCorners(img, (self.cb_w,self.cb_h), corners2,ret)
                    cv2.imwrite(self.projected_destination + "/corners_" + str(i) + ".jpg", img)

                # Assign to image set
                self.images.append(self.imageInitial[i])
            else:
                # Add to error list
                errorList.append(i)
                unfound += 1


        print(datetime.now().strftime('%H:%M:%S') + " ==> Found %s checkerboards, didn't find %s checkerboards." % (found, unfound))
        # If errors found delete files and re-order
        if len(errorList) > 0:
            # Remove all error files in errorList and append directories to be re-ordered
            foldersToReOrder = []
            for image in errorList:
                fileName = Path(self.imageInitial[image]).stem
                allFiles = glob.glob(self.output_destination + "*\\*" + fileName + '.*', recursive=True)
                for file in allFiles:
                    print(datetime.now().strftime('%H:%M:%S') + " ==> Deleting file " + str(file))
                    os.remove(file)
                    if os.path.dirname(file) not in foldersToReOrder:
                        foldersToReOrder.append(os.path.dirname(file))

            print(datetime.now().strftime('%H:%M:%S') + " ==> Deleting undetectable checkerboards complete.")
            # Re order directories
            x = re.compile("[0-9]")
            for folder in foldersToReOrder:
                print(datetime.now().strftime('%H:%M:%S') + " ==> Updating folder " + str(folder), end='\r')
                counters = 0
                characters = ""
                for file in glob.glob(folder + "\\*.*"):
                    # Get file name
                    fileName = Path(file).stem
                    # Check if current file is numbered, if not ignore
                    numbered = len(re.findall(x, fileName))
                    # If less than 7, do not renumber
                    if numbered < 8:
                        continue
                    # Check for extra text
                    extraCharacters = len(fileName) - 8
                    if extraCharacters > 0:
                        if fileName[0:extraCharacters] == characters:
                            os.rename(file, folder + "\\" + characters + str(counters).rjust(8, '0') + Path(file).suffix)
                            counters = counters + 1
                        else:
                            # Reset for new character
                            characters = fileName[0:extraCharacters]
                            counters = 0
                            # Begin assignment
                            os.rename(file, folder + "\\" + characters + str(counters).rjust(8, '0') + Path(file).suffix)
                            # Update for next increment
                            counters += 1
                    else:
                        os.rename(file, folder + "\\" + str(counters).rjust(8, '0') + Path(file).suffix)
                        counters = counters + 1
            print(datetime.now().strftime('%H:%M:%S') + " ==> Folders updated.")

        #
        sampleImage = cv2.imread(self.images[0])
        # Get the dimensions of the image
        height, width = sampleImage.shape[:2]

        # Extracting camera parameters
        # distCoeffs Output vector of distortion coefficients [k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,taux,tauy] of 4, 5, 8, 12 or 14 elements.
        if self.config['calibration']['est_intrinsic'] == "True":
            print("==> Estimating intrinsic parameters")
            retval, intrinsicMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, flags=cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL)
            # For defining intrin
        elif self.config['calibration']['est_intrinsic'] == "False":
            print("==> Intrinsic parameters provided")
            retval, intrinsicMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], self.customIntrin, None, flags=cv2.CALIB_USE_INTRINSIC_GUESS +  cv2.CALIB_FIX_PRINCIPAL_POINT + cv2.CALIB_FIX_FOCAL_LENGTH )

        # save dist
        np.savetxt(self.axyb_destination + "/distCoeffs.txt", distCoeffs,  fmt='%.8f')

        # Refine camera matrix
        # Returns optimal camera matrix and a rectangular region of interest
        optimalIntrinsicMatrix, roi = cv2.getOptimalNewCameraMatrix(intrinsicMatrix, distCoeffs, (width,height), 1, (width,height))
        # Save intrinsic
        np.savetxt(self.axyb_destination + "/intrinsicOptimised.txt", optimalIntrinsicMatrix,  fmt='%.8f')
        np.savetxt(self.axyb_destination + "/intrinsic.txt", intrinsicMatrix,  fmt='%.8f')


        ###
        # ESTIMATE ERROR
        ###

        mean_error = 0
        # Estimate accuracy
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], intrinsicMatrix, distCoeffs)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print(datetime.now().strftime('%H:%M:%S') + f"==> Estimated error (pixels): {round(mean_error/len(objpoints),4)}")

        images = glob.glob(self.image_destination + '*.jpg')

        if self.distort == "True":
            for fname in images:
                img = cv2.imread(fname)
                # undistort
                dst = cv2.undistort(img, intrinsicMatrix, distCoeffs, None, optimalIntrinsicMatrix)
                # crop the image
                x, y, w, h = roi
                dst = dst[y:y+h, x:x+w]
                # Get file name
                out_name = os.path.basename(fname)
                # Save cropped
                cv2.imwrite(self.projected_destination + out_name, dst)
        else:
            for fname in images:
                img = cv2.imread(fname)
                out_name = os.path.basename(fname)
                cv2.imwrite(self.projected_destination + out_name, img)
        ###
        # OUTPUT VERIFICATION
        ###
        counter = 0
        # Point to project
        point = [0,0,0,1]
        # Export the camera matrices
        for i in range(0, len(images)):
            # Check if checkerboard is visible
            file_name = str(i).zfill(8)

            # The rotation and translation of the camera
            A = np.append(cv2.Rodrigues(rvecs[counter])[0],tvecs[counter],axis=1)

            # Calculate projection matrix
            projection_matrix = np.dot(intrinsicMatrix, A)
            projection_matrix = np.append(projection_matrix, [[0, 0, 0, 1]], axis=0)

            # Calculate optimal
            projection_matrix_optimal = np.dot(optimalIntrinsicMatrix, A)
            projection_matrix_optimal = np.append(projection_matrix_optimal, [[0, 0, 0, 1]], axis=0)

            # Camera matrix
            A = np.append(A, [[0, 0, 0, 1]], axis=0)
            # Save files
            np.savetxt(self.axyb_destination + "/A" + file_name + ".txt", A,  fmt='%.8f')
            np.savetxt(self.projection_destination + file_name + ".txt", projection_matrix,  fmt='%.8f')

            # If testing output projected origin based on camera estimate
            if self.config['general']['testing'] == "True":
                # Load image as PIL
                input_image = Image.open(self.projected_destination + "\\"  + file_name  + ".jpg")

                # Project point to camera using original
                v = projection_matrix @ point # dot product
                x = int(v[0] / v[2])
                y = int(v[1] / v[2])
                # Draw ellipse
                draw = ImageDraw.Draw(input_image)
                draw.ellipse((x-5, y-5, x+5, y+5), fill=(0,255,0))

                # Saving the final output
                input_image.save(self.projected_destination + "\\"  + file_name + ".jpg" )
                input_image.close()

                ##################################################################

            counter = counter + 1




    def estimateA(self, imageFolder, calibrationFolder, targetFolder):
        Y = np.loadtxt(calibrationFolder + "Y.txt")
        X = np.loadtxt(calibrationFolder + "X.txt")
        intrinsic = np.loadtxt(calibrationFolder + "intrinsic.txt")
        #ttCenter = np.loadtxt(calibrationFolder + "center.txt")
        rError = np.loadtxt(calibrationFolder + '\\rError.txt')
        tError = np.loadtxt(calibrationFolder + '\\tError.txt')
        distCoeffs = np.loadtxt(calibrationFolder + '\\distCoeffs.txt')
        optimal = np.loadtxt(calibrationFolder + '\\intrinsicOptimised.txt')

        for filename in os.listdir(imageFolder):
            f = os.path.join(imageFolder,filename)
            if os.path.isfile(f):
                B = np.loadtxt(calibrationFolder + "B" + filename.replace(".jpg",".txt"))

                YFinal = Y

                # Calculate and save A if testing, else not needed
                A = (X @ functions.matrix_inverse((YFinal @ B)))

                # Output projection (for testing purposes only)
                projectPoint =np.array([0,0,0,1])

                # Estimate projection matrix and save
                matrix = (intrinsic @ A[0:3,:])

                # Project point to camera
                v = (matrix @ projectPoint) # dot product
                x = int(v[0] / v[2])
                y = int(v[1] / v[2])

                # Load image as PIL
                input_image = Image.open(self.projected_destination + "\\" + filename)

                # Draw ellipse
                draw = ImageDraw.Draw(input_image)
                draw.ellipse((x-5, y-5, x+5, y+5), fill=(125,125,125))

                # Estimate using error
                # Adjust for error if exists
                if rError is not None and tError is not None:
                    A[0:3,3] = A[0:3,3] + tError
                    A[0:3,0:3] = A[0:3,0:3] + rError
                    # Estimate projection matrix and save
                    matrixErr = np.dot(intrinsic, A[0:3,:])

                    # Output projection (for testing purposes only)
                    projectPoint =np.array([0,0,0,1])

                    # Project point to camera
                    v = (matrixErr @ projectPoint) # dot product
                    x = int(v[0] / v[2])
                    y = int(v[1] / v[2])

                    # Draw ellipse
                    draw = ImageDraw.Draw(input_image)
                    draw.ellipse((x-5, y-5, x+5, y+5), fill=(0,0,255))

                # Saving the final output
                input_image.save(self.projected_destination + "\\" + filename)
                input_image.close()



    def adjust_gamma(self, image, gamma=1.0):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                          for i in np.arange(0, 256)]).astype("uint8")

        # apply gamma correction using the lookup table
        return cv2.LUT(image, table)

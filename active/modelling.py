"""
Calibration of the robot (UR5)
Created by Jonathon Gibbs, please reference if used.

www.jonathongibbs.co.uk
https://github.com/pszjg
"""

import cv2
import time
import os.path
import numpy as np
from calibration import functions
from PIL import Image, ImageDraw

class ImageAcquisition():
    def __init__(self):
        pass

    def initialise(self, robot, cam, turntable, config):
        # Robot, camera and turntable
        self.cam = cam
        self.robot = robot
        self.turntable = turntable
        self.config = config
        self.distCoeffs = None
        self.intrinsicOptimised = None

        # Check for existance of calibration
        self.calibrationPath = self.config['calibration']['working_dir'] + "\\axyb\\"
        if os.path.exists(self.calibrationPath + "X.txt") and os.path.exists(self.calibrationPath + "Y.txt") and os.path.exists(self.calibrationPath + "intrinsic.txt"):
            # Load X and Y
            if not os.path.exists(self.calibrationPath + "center.txt"):
                print("==> Error: Turntable not calibrated, using (0,0,0)")
                with open(self.calibrationPath + "center.txt", 'w+') as f:
                    f.write('0\n0\n0\n')

            self.X = np.loadtxt(self.calibrationPath + "X.txt")
            self.Y = np.loadtxt(self.calibrationPath + "Y.txt")
            self.rError = np.loadtxt(self.calibrationPath + '\\rError.txt')
            self.tError = np.loadtxt(self.calibrationPath + '\\tError.txt')
            self.intrinsic = np.asmatrix(np.loadtxt(self.calibrationPath + "intrinsic.txt"))
            self.ttCenter = np.loadtxt(self.calibrationPath + "center.txt")
            self.ttCenter[2] = 0

            # If optimal camera matrix has been estimated load them here
            if os.path.exists(self.calibrationPath + "intrinsicOptimised.txt") and os.path.exists(self.calibrationPath + "distCoeffs.txt"):
                self.intrinsicOptimised = np.asmatrix(np.loadtxt(self.calibrationPath + "intrinsicOptimised.txt"))
                self.distCoeffs = np.array([np.loadtxt(self.calibrationPath + "distCoeffs.txt")])

            # Setup storage folders
            self.imageFolder = self.config['model']['working_dir'] + '\\visualize\\'
            self.txtFolder = self.config['model']['working_dir'] + '\\txt\\'
            self.degreesPerRot = int(self.config['model']['degreesPerRot'])
            # Create folders
            if not os.path.exists(self.imageFolder): os.makedirs(self.imageFolder)
            if not os.path.exists(self.txtFolder):   os.makedirs(self.txtFolder)

            # If testing
            if self.config['general']['testing'] == "True":
                # Image output
                self.testFolder = self.config['model']['working_dir'] + '\\testing\\'
                if not os.path.exists(self.testFolder):   os.makedirs(self.testFolder)
                # Original estimate of A output
                self.aFolder = self.config['model']['working_dir'] + '\\A\\'
                if not os.path.exists(self.aFolder):     os.makedirs(self.aFolder)
        else:
            print("==> Error: Calibration files (X.txt and Y.txt) have not been found in")
            print("==> .... " + self.calibrationPath)

    def calculatePositions(self):
        # TODO
        np.set_printoptions(formatter={'float': lambda x: "%.5f" % (x,)})
        tempPath = self.config['calibration']['working_dir'] + "\positionLog_cap.txt"
        robotPositions = np.loadtxt(tempPath, delimiter=',')
        return robotPositions

    # Temporary manual positions
    def run(self):
        print("==> Running 3D modelling")
        # Reset positions
        self.turntable.Home()
        self.robot.home()
        # Set save location for camera
        original = self.cam.path
        self.cam.path = self.imageFolder

        # TODO: Automatically generate these
        robotPositions = self.calculatePositions()

        # TODO: this is a temporary solution of 30 degree rotations:
        rotations = int(360 / self.degreesPerRot)

        # Collect data
        for i in range(0, rotations):
            Y = np.loadtxt(self.calibrationPath + "Y.txt")
            # Account for rotation
            degrees = self.degreesPerRot * i
            # Move turntable
            self.turntable.GoTo(degrees)

            ###############################
            ####### Y Calculations ########
            ###############################
            # If rotation is not zero then adjust
            if degrees == 0:
                print("Zero degrees")
            else:
                # Adjust for robot positions
                degrees = degrees * -1
                print("==> Calculating rotation of robot (" + str(degrees) + ")")
                # Convert to rads
                radians = (degrees * 0.0174532925)
                # get rotation of Y estimate
                estY = np.dot(Y, functions.rotZ(radians))
                # Difference in translation
                cPoint = self.ttCenter * -1
                cPoint = np.append(cPoint, [1])
                point = np.dot(functions.rotZ(radians), cPoint)
                # calculate translation back to origin
                point[0:3] = point[0:3] + self.ttCenter

                # translate to origin
                tempVec =  np.dot(functions.matrix_inverse(Y), point)
                # Create Y
                estY[0:3,3] = tempVec[0:3]

                # Invert back to final Y
                Y = functions.matrix_inverse(estY)
            ###############################
            ##### End Y Calculations ######
            ###############################

            # Iterate positions
            for position in robotPositions:
                A = None
                # Move robot to position
                self.robot.move(position)
                # Capture image
                currentFile = self.cam.capture()

                # Check that image was found
                if currentFile == False:
                    continue
                # Calculate robot transformation matrix and save
                B = self.robot.angles_to_transformation()
                np.savetxt(self.testFolder + "B" + currentFile + '.txt', B, fmt='%1.5f')

                # Calculate and save A
                A = np.dot(self.X, functions.matrix_inverse(np.dot(Y, B)))
                np.savetxt(self.aFolder + currentFile + '.txt', A, fmt='%1.5f')

                # Estimate projection matrix and save
                matrix = np.dot(self.intrinsic, A[0:3,:])
                np.savetxt(self.txtFolder + currentFile + '.txt', matrix, fmt='%1.5f')

                # Output projection (for testing purposes only)
                projectPoint =np.array([0,0,0,1])

                # Project point to camera
                v = (matrix @ projectPoint) # dot product

                x = int(v[0,0] / v[0,2])
                y = int(v[0,1] / v[0,2])

                # Import an image from directory:
                input_image = Image.open(self.imageFolder + currentFile + '.jpg')
                # Draw ellipse
                draw = ImageDraw.Draw(input_image)
                draw.ellipse((x-5, y-5, x+5, y+5), fill=(255,0,0,0))

                # Adjust for error if exists
                if self.rError is not None and self.tError is not None:
                    A[0:3,3] = A[0:3,3] + self.tError
                    A[0:3,0:3] = A[0:3,0:3] + self.rError

                    # Estimate projection matrix and save
                    matrix = np.dot(self.intrinsic, A[0:3,:])

                    # Output projection (for testing purposes only)
                    projectPoint =np.array([0,0,0,1])

                    # Project point to camera
                    v = (matrix @ projectPoint) # dot product
                    x = int(v[0,0] / v[0,2])
                    y = int(v[0,1] / v[0,2])

                    # Draw ellipse
                    draw.ellipse((x-5, y-5, x+5, y+5), fill=(0,0,255,0))

                input_image.save(self.testFolder + currentFile + '.jpg')
                input_image.close()

        self.cam.path = original

if __name__ == "__main__":
    # Start application
    A = ImageAcquisition()
    A.start()

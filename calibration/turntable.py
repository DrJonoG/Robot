"""
Calibration of the Turntable (Generically applicable)
Created by Jonathon Gibbs, please reference if used.

www.jonathongibbs.co.uk
https://github.com/pszjg
"""
from calibration import functions as functions
from active import calibration as activePositions
from calibration import camera as CamCalibration
import os
import math
import numpy as np

class calibrateTurntable(object):
    def __init__(self, config):
        self.config = config
        self.rotations = int(self.config['calibration']['tt_rotations'])

        # Make base path
        self.path = self.config['calibration']['working_dir'] + '\\turntable\\'
        if not os.path.exists(self.path):
            os.makedirs(self.path)

        # Make path for each rotation:
        for i in range(0, self.rotations):
            if not os.path.exists(self.path + str(i)):
                os.makedirs(self.path + str(i))

    def initialise(self, robot, cam, turntable):
        self.robot = robot
        self.cam = cam
        self.turntable = turntable

    def calibrate(self):
        # Reset robot position
        self.robot.home()
        # Store original camera path
        originalPath = self.cam.path
        # Get robot positions
        robotPositions = activePositions.turntableCalibration(self.robot, self.cam, 25, self.config['calibration']['working_dir'])
        degreesPerRot = round(360 / self.rotations, 2)
        for i in range(0, self.rotations):
            # Update camera path
            self.cam.path = self.path + '\\' + str(i) + '\\images\\'
            camCalibration = CamCalibration.calibrateCamera(self.path + str(i) + '\\', self.config)
            # for each rotation perform calibration
            print(degreesPerRot * i)
            for position in robotPositions:
                self.turntable.GoTo(degreesPerRot * i)
                # Move robot to position
                self.robot.move(position)
                # Capture image
                _ = self.cam.capture()
            # Perform calibration on the collected images
            camCalibration.calibrate()
        # Estimate the center of the turntable using the collection of calibrations
        self.estimateCenter()
        # Reset to home position
        self.turntable.Home()

    def estimateCenter(self):
        # Check if enough
        if self.rotations < 4:
            print("==> Error: more rotations are required for turntable calibration.")
            exit()

        # Number of lines (start and end creates one line thus rotations / 2)
        numLines = int(self.rotations / 2)

        # Calculate average intersection
        intersection = 0

        # image count
        imageCount = 0

        # Loop through each image (robot position)
        for filename in os.listdir(self.path + "\\0\\axyb\\"):
            if 'intrinsic' in filename or 'dist' in filename:
                continue
            # Empty arrays
            PA = np.empty([numLines, 3])
            PB = np.empty([numLines, 3])

            # Loop through turntable rotations and load files
            for i in range(0, numLines):
                startDir = self.path + "\\" + str(i) + "\\axyb\\"
                endDir = self.path + "\\" + str(i + numLines) + "\\axyb\\"

                # Skip intrinsic matrix


                # Check same file exists in end dir
                if not os.path.exists(endDir + filename):
                    print("==> Error calibrating tunrtable. End rotation not found")
                    exit()

                # Load files and obtain translation of each file
                PA[i] = np.transpose(functions.matrix_inverse(np.asmatrix(np.loadtxt(startDir + filename)))[0:3,3])
                PB[i] = np.transpose(functions.matrix_inverse(np.asmatrix(np.loadtxt(endDir + filename)))[0:3,3])

            # Update count
            imageCount += 1

            # Difference
            Si = PB - PA

            # Normalise
            for x in range(0, numLines):
                PAn = math.sqrt((Si[x, 0] * Si[x, 0]) + (Si[x, 1] * Si[x, 1]) + (Si[x, 2] * Si[x, 2]))
                Si[x, 0] = Si[x, 0] / PAn;
                Si[x, 1] = Si[x, 1] / PAn;
                Si[x, 2] = Si[x, 2] / PAn;

            # Normalised columns
            nx = Si[0:, 0]
            ny = Si[0:, 1]
            nz = Si[0:, 2]

            SXX = 0
            SYY = 0
            SZZ = 0;

            for z in range(0, len(nx)):
                SXX += (pow(nx[z], 2) - 1)
                SYY += (pow(ny[z], 2) - 1)
                SZZ += (pow(nz[z], 2) - 1)

            SXY = np.dot(nx, ny)
            SXZ = np.dot(nx, nz)
            SYZ = np.dot(ny, nz)

            S = np.array([
                [SXX, SXY, SXZ],
                [SXY, SYY, SYZ],
                [SXZ, SYZ, SZZ]
            ])


            # Normalised columns
            PAx = PA[0:, 0]
            PAy = PA[0:, 1]
            PAz = PA[0:, 2]

            CX1 = (PAx[0] * (pow(nx[0], 2) - 1)) + (PAx[1] * (pow(nx[1], 2) - 1)) + (PAx[2] * (pow(nx[2], 2) - 1)) + (PAx[3] * (pow(nx[3], 2) - 1));
            CX2Holder = np.multiply(PAy,np.multiply(nx,ny))
            CX2 = CX2Holder[0] + CX2Holder[1] + CX2Holder[2] + CX2Holder[3];
            CX3Holder = np.multiply(PAz,np.multiply(nx,nz))
            CX3 = CX3Holder[0] + CX3Holder[1] + CX3Holder[2] + CX3Holder[3];
            CX = CX1 + CX2 + CX3;

            CY1Holder = np.multiply(PAx,np.multiply(nx,ny))
            CY1 = CY1Holder[0] + CY1Holder[1] + CY1Holder[2] + CY1Holder[3];
            CY2 = (PAy[0] * (pow(ny[0], 2) - 1)) + (PAy[1] * (pow(ny[1], 2) - 1)) + (PAy[2] * (pow(ny[2], 2) - 1)) + (PAy[3] * (pow(ny[3], 2) - 1));
            CY3Holder = np.multiply(PAz,np.multiply(ny,nz))
            CY3 = CY3Holder[0] + CY3Holder[1] + CY3Holder[2] + CY3Holder[3];
            CY = CY1 + CY2 + CY3;

            CZ1Holder = np.multiply(PAx,np.multiply(nx,nz))
            CZ1 = CZ1Holder[0] + CZ1Holder[1] + CZ1Holder[2] + CZ1Holder[3];
            CZ2Holder = np.multiply(PAy,np.multiply(ny,nz))
            CZ2 = CZ2Holder[0] + CZ2Holder[1] + CZ2Holder[2] + CZ2Holder[3];
            CZ3 = (PAz[0] * (pow(nz[0], 2) - 1)) + (PAz[1] * (pow(nz[1], 2) - 1)) + (PAz[2] * (pow(nz[2], 2) - 1)) + (PAz[3] * (pow(nz[3], 2) - 1));
            CZ = CZ1 + CZ2 + CZ3;

            C = [CX, CY, CZ]

            intersection = intersection + np.linalg.solve(S, C)

        intersection = intersection / imageCount

        # Set output options and save
        np.savetxt(self.config['calibration']['working_dir'] + "\\axyb\\center.txt", intersection, delimiter=',', fmt='%f')

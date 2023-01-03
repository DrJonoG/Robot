from hardware import camera as C
from hardware import robot as R
from hardware import turntable as T
from calibration import functions
from calibration import camera as CamCalibration
from calibration import axyb as AXYB
from calibration import turntable as TTCalibration
from active import modelling as Model
from io_func import read_write as IO
from io_func import pmvs as PMVS
from io_func import filter as Filter
from calibration import functions
from threading import Thread
from PIL import Image, ImageDraw
import glob
import configparser
import numpy as np
import shutil
import pathlib
import os

class main(object):
    def __init__(self, path, config):
        self.cam_thread = None
        self.robot_thread = None
        self.turntable_thread = None
        self.main_thread = None

        self.config = config
        self.config = configparser.ConfigParser()
        self.config.read("./config.ini")

        self.calibrationPath = self.config['calibration']['working_dir'] + "\\axyb\\"
        self.cam = C.fetchClass(self.config['hardware']['camera'], self.config)
        self.robot = R.fetchClass(self.config['hardware']['robot'], self.config)
        self.turntable = T.fetchClass(self.config['hardware']['turntable'], self.config)
        self.camCalibration = CamCalibration.calibrateCamera(self.config['calibration']['working_dir'], self.config)
        self.ttCalibration = TTCalibration.calibrateTurntable(self.config)
        self.modelling = Model.ImageAcquisition()
        self.PMVS = PMVS.PMVS(self.config['model']['working_dir'])

        self.intrinsic = np.asmatrix(np.loadtxt(self.calibrationPath + "intrinsic.txt"))
        self.ttCenter = np.loadtxt(self.calibrationPath + "center.txt")

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
            # Positions for calibrating
            self.existingA = self.config['model']['working_dir'] + '\\existingA\\'
            if not os.path.exists(self.existingA):     os.makedirs(self.existingA)

        print("==> Working calibration directory: " + self.config['calibration']['working_dir'])
        print("==> Working model directory: " + self.config['model']['working_dir'])

        self.cam.connect()
        self.cam.livestream()
        self.robot.connect()
        self.turntable.connect()

    def checkCalibration(self, degrees):
        self.turntable.connect()
        self.turntable.GoTo(degrees)

    def calculatePositions(self):
        # TODO
        np.set_printoptions(formatter={'float': lambda x: "%.5f" % (x,)})
        tempPath = self.config['calibration']['working_dir'] + "\positionLog.txt"
        robotPositions = np.loadtxt(tempPath, delimiter=',')
        return robotPositions

    def capture(self, degreesPerRot):
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
        rotations = int(360 / degreesPerRot)

        # Collect data
        for i in range(0, rotations):
            # Account for rotation
            degrees = degreesPerRot * i
            print("==> Collecting for " + str(degrees) + " degrees.")
            # Move turntable
            self.turntable.GoTo(degrees)
            # Iterate positions
            counter = 0
            for position in robotPositions:
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
                # Load the original A file from camera calibration
                aMatrix = np.loadtxt(self.existingA + "A0000000" + str(counter) + '.txt')
                # If degrees are greeater than 0 then adjust A position
                if degrees > 0 & degrees < 360:
                    aMatrix = functions.matrix_inverse(aMatrix)
                    # Convert to rads
                    radians = ((degrees* - 1) * 0.0174532925)
                    # get rotation of Y estimate
                    estTxt = (aMatrix @ functions.rotZ(radians))
                    # Difference in translation
                    cPoint = self.ttCenter * -1
                    cPoint = np.append(cPoint, [1])
                    point = (functions.rotZ(radians) @ cPoint)
                    # calculate translation back to origin
                    point[0:3] = point[0:3] + (cPoint[0:3]*-1)
                    # translate to origin
                    aMatrix = functions.matrix_inverse(aMatrix)
                    tempVec =  (aMatrix @ point)
                    # Create Y
                    estTxt[0:3,3] = tempVec[0:3]
                    # Assign matrix
                # Save files
                np.savetxt(self.aFolder + currentFile + '.txt', aMatrix, fmt='%1.5f')
                projectionMatrix = (self.intrinsic @ aMatrix[0:3,:])
                np.savetxt(self.txtFolder + currentFile + '.txt', projectionMatrix, fmt='%1.5f')

                # Output projection (for testing purposes only)
                projectPoint =np.array([0,0,0,1])
                v = (projectionMatrix @ projectPoint)
                x = int(v[0,0] / v[0,2])
                y = int(v[0,1] / v[0,2])
                # Import an image from directory and draw the world origin
                input_image = Image.open(self.imageFolder + currentFile + '.jpg')
                draw = ImageDraw.Draw(input_image)
                draw.ellipse((x-5, y-5, x+5, y+5), fill=(255,0,0,0))
                input_image.save(self.testFolder + currentFile + '.jpg')
                input_image.close()

                counter = counter + 1








    def test(self, degrees):
        folder = "I:\\calibration\\test\\"
        aMatrix = "I:\\calibration\\test\\A00000000.txt"
        center = "I:\\calibration\\axyb\\center.txt"
        intrinsic = "I:\\calibration\\axyb\\intrinsic.txt"
        original = "I:\\calibration\\test\\original.jpg"
        rotated = "I:\\calibration\\test\\rotated.jpg"

        intrinsic = np.loadtxt(intrinsic)
        cPoint = np.loadtxt(center)
        aMatrix = np.loadtxt(aMatrix)
        aMatrix = functions.matrix_inverse(aMatrix)

        # Adjust for robot positions
        degrees = degrees * - 1
        print("==> Calculating rotation of robot (" + str(degrees) + ")")
        # Convert to rads
        radians = (degrees * 0.0174532925)
        # get rotation of Y estimate
        estTxt = (aMatrix @ functions.rotZ(radians))
        # Difference in translation
        cPoint = cPoint * -1
        cPoint = np.append(cPoint, [1])

        point = (functions.rotZ(radians) @ cPoint)

        # calculate translation back to origin
        point[0:3] = point[0:3] + (cPoint[0:3]*-1)

        # translate to origin
        aMatrix = functions.matrix_inverse(aMatrix)
        tempVec =  (aMatrix @ point)
        # Create Y
        estTxt[0:3,3] = tempVec[0:3]
        # Save matrix
        aMatrix = estTxt
        np.savetxt("I:\\calibration\\test\\A00000000_rotated.txt", aMatrix,  fmt='%1.5f')
        # Estimate projection matrix and save
        matrix = (intrinsic @ aMatrix[0:3,:])

        # Output projection (for testing purposes only)
        projectPoint =np.array([0,0,0,1])
        # Project point to camera
        v = (matrix @ projectPoint) # dot product

        x = int(v[0] / v[2])
        y = int(v[1] / v[2])
        print(x, y)
        # Import an image from directory:
        input_image = Image.open(rotated)
        # Draw ellipse
        draw = ImageDraw.Draw(input_image)
        draw.ellipse((x-5, y-5, x+5, y+5), fill=(255,0,0,0))
        input_image.save(rotated.replace(".jpg","_new.jpg"))
        input_image.close()

if __name__ == "__main__":
    # Start application
    interface = main("./", "./config.ini")

    interface.capture(36)

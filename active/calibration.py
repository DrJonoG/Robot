from io_func import read_write as IO
import time
import random
import numpy as np
# Captures
def capture(robot, cam, turntable, image_path, axyb_path, positionPath, positions):

    # Inform user
    print("==> Calibrating. Saving files to " + positionPath)
    # Store original camera path
    store_path = cam.path
    # Update camera path
    cam.path = image_path

    # Reset to home positon
    robot.home()
    # Temporary manual positions

    # TODO
    np.set_printoptions(formatter={'float': lambda x: "%.5f" % (x,)})
    robotPositions = np.loadtxt(positions, delimiter=',')

    #robotPositionsAdjust = []
    #for i in range(0,2):
    #    for robotPos in robotPositions:
    #        robotPositionsAdjust.append(robotPos + np.random.uniform(-0.08,0.08, 6))
    #robotPositions = robotPositions + robotPositionsAdjust

    # Go to each position and capture images
    for i in range(0, len(robotPositions)):
        # Move robot to position
        robot.move(robotPositions[i])
        # Pause for robot to stop moving
        time.sleep(4)
        # Capture image
        currentFile = cam.capture(True)
        # Calculate robot transformation matrix
        robotMatrix = robot.angles_to_transformation()
        # Write matrix
        IO.wMatrix(robotMatrix, currentFile, axyb_path, 'B')

    # Update camera path
    cam.path = store_path


# Function to obtain positions for the calibration of the turntable
# Seperate method to acquistion due to different goals
def turntableCalibration(robot, cam, maxpositions, positionPath, positions):
    # Todo actively search for checkerboard positions
    np.set_printoptions(formatter={'float': lambda x: "%.5f" % (x,)})
    robotPositions = np.loadtxt(positions, delimiter=',')
    # Limit the return
    return robotPositions#[0:maxpositions]

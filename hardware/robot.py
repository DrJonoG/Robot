from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import numpy as np
import os
import math
import socket
import time
from colorama import init as colorama_init
from colorama import Fore
from colorama import Style

colorama_init()

def fetchClass(robotType, config):
    if robotType.lower() == 'ur5' or robotType.lower() == 'ur10' or robotType.lower() == 'ur':
        return UR(config)

# Base class of the robot
class RobotBase(object):
    def __init__(self, config=None):
        self.connected = False
        self.moving = False
        self.position = None
        self.s = None
        self.path = None
        self.rtde_c = None
        self.rtde_r = None
        self.config = config

    def connect(self, host, port):
        raise NotImplementedError("robot.connect has not been implemented")

    def home(self):
        raise NotImplementedError("robot.home has not been implemented")

    def move(self, position,seconds=0):
        raise NotImplementedError("robot.move has not been implemented")

    def send(self, command):
        raise NotImplementedError("robot.send has not been implemented")

    def disconnect(self):
        raise NotImplementedError("robot.disconnect has not been implemented")

    def get_position(self):
        raise NotImplementedError("robot.get_position has not been implemented")

    def get_kinematics(self):
        raise NotImplementedError("robot.get_kinematics has not been implemented")

    def angles_to_transformation(self):
        forwardKinematics = self.get_kinematics()

        # Calculate the transpose
        T = np.multiply(forwardKinematics[0:3], 1000)

        # Calculate the angle of rotation
        angleOfRotation = math.sqrt(math.pow(forwardKinematics[3], 2) + math.pow(forwardKinematics[4], 2) + math.pow(forwardKinematics[5], 2))

        # Calculate axisangle angle
        axisAngle = np.append(forwardKinematics[3:6] / angleOfRotation, angleOfRotation)

        # Rotation matrix
        rotationMatrix = self.matrix_from_axis_angle(axisAngle)

        # Complete
        matrix = np.column_stack((rotationMatrix, np.transpose(T)))
        matrix = np.vstack([matrix, [0,0,0,1]])
        return matrix

    def matrix_from_axis_angle(self, a1, normalised=True):
        c = math.cos(a1[3]);
        s = math.sin(a1[3]);
        t = 1.0 - c;

        # if axis is not already normalised then
        if not normalised:
            magnitude = math.sqrt(a1[0]*a1[0] + a1[1]*a1[1] + a1[2]*a1[2]);
            if (magnitude==0):
                print("==> Invalid values for axis angle")
                return
            a1[0] /= magnitude;
            a1[1] /= magnitude;
            a1[2] /= magnitude;

        m00 = c + a1[0] * a1[0] * t;
        m11 = c + a1[1] * a1[1] * t;
        m22 = c + a1[2] * a1[2] * t;

        tmp1 = a1[0] * a1[1] * t;
        tmp2 = a1[2] * s;

        m10 = tmp1 + tmp2;
        m01 = tmp1 - tmp2;

        tmp1 = a1[0] * a1[2] * t;
        tmp2 = a1[1] * s;

        m20 = tmp1 - tmp2;
        m02 = tmp1 + tmp2; tmp1 = a1[1] * a1[2] * t;

        tmp2 = a1[0] * s;

        m21 = tmp1 + tmp2;
        m12 = tmp1 - tmp2;

        rotationMatrix = np.empty([3,3])
        #DenseMatrix matrix = new DenseMatrix(3, 3);
        rotationMatrix[0, 0] = m00;
        rotationMatrix[0, 1] = m01;
        rotationMatrix[0, 2] = m02;

        rotationMatrix[1, 0] = m10;
        rotationMatrix[1, 1] = m11;
        rotationMatrix[1, 2] = m12;

        rotationMatrix[2, 0] = m20;
        rotationMatrix[2, 1] = m21;
        rotationMatrix[2, 2] = m22;

        return rotationMatrix;

    def msg_connected(self):
        print("==> Robot connected")
        self.connected = True

    def msg_disconnected(self):
        print("==> Robot disconnected")
        self.connected = False


class MobileRobot(RobotBase):
    pass

class UR(RobotBase):
    def __init__(self, config):
        self.connected = False
        self.moving = False
        self.position = None
        self.s = None
        self.config = config

    def connect(self):
        if(self.connected):
            return True
        else:
            try:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.connect((self.config['robot']['host'], int(self.config['robot']['port'])))
                # RTDE connect
                self.rtde_c = RTDEControl(self.config['robot']['host'])
                self.rtde_r = RTDEReceive(self.config['robot']['host'])
                # Wait 0.05 seconds because the robot communication speed is 125 Hz = 0.008 seconds
                time.sleep(0.05)
                # Connected
                self.msg_connected()
            except socket.error as msg:
                self.connected = False
                print("Caught exception socket.error : %s" % msg)
            except TypeError as msg:
                self.connected = False
                print("Type Error: %s" % msg)


    def home(self):
        self.move([0,-1.57,0,-1.57,0,0])

    def move(self, position, wait=True):
        self.rtde_c.moveJ(position, float(self.config['robot']['speed']), float(self.config['robot']['accel']))
        # wait for move to finish
        if wait:
            while not self.rtde_c.isSteady():
                time.sleep(0.3)

    def send(self, command):
        if not self.connected:
            raise ValueError('Not connected, please ensure you are connected to the robot')
            return
        try:
            self.s.send(command.encode('utf8'))
        except:
            raise ValueError('Unable to execute ' + command)

    def disconnect(self):
        self.msg_disconnected()
        self.s.close()

    def get_position(self):
        return rtde_r.getActualQ()

    def get_kinematics(self):
        return np.array(self.rtde_c.getForwardKinematics())

    # Open the gripper - This is a temporary hack to open gripper.
    def open(self):
        print("==> Opening gripper")
        f = open("./hardware/open.script", "r")
        l = f.read(1024)

        while(l):
        	self.send(l)
        	l = f.read(1024)
        time.sleep(0.2)

    # Close the gripper - This is a temporary hack to close gripper.
    def close(self):
        print("==> Closing gripper")
        f = open("./hardware/close.script", "r")
        l = f.read(1024)

        while(l):
        	self.send(l)
        	l = f.read(1024)
        time.sleep(0.2)

    # Calculate the distance to singularity based on current position
    def singularity(self):
         # WRIST
         actual_q = self.rtde_r.getActualQ()
         q_5 = actual_q[4]
         # calculate distance to wrist singularity.
         d_w = math.sin(q_5) ** 2
         # HEAD
         current_pose = self.rtde_r.getActualTCPPose()
         x = current_pose[0]
         y = current_pose[1]
         # calculate distance to head singularity.
         d_h = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
         # ELBOW
         # Simply how close q3 is to zero.
         d_e = actual_q[2]

         return [d_w, d_h, d_e]

    # Used to log positions that the user manually moves the robot too, ideal for setting up calibration
    def manual_positions(self):
        os.makedirs(self.config['model']['working_dir'], exist_ok=True)
        # define error colors
        CRED = '\033[91m'
        CEND = '\033[0m'

        print("==> Record positions, enter to log position, q to quit.")
        positionCount = 0
        while True:    # infinite loop
            # Turn on manual drive
            self.rtde_c.teachMode()
            time.sleep(0.2)
            # Wait for input
            n = input("")
            if n == "q":
                self.rtde_c.stopScript()
                break
            # Joint angles
            with open(self.config['calibration']['positions'], "a+") as f:
                stringOut = [ '%.5f' % elem for elem in self.rtde_r.getActualQ() ]
                print("==> Position (# " + str(positionCount) + ") " + str(stringOut).replace("'","")  + " saved.")

                s = self.singularity()
                if s[0] < 0.1 or s[1] < 0.1 or (s[2] < 0.09 and s[2] > -0.09):
                    print('==>' + CRED + ' Singularity: {:f}'.format(s[0]) + " " + '{:f}'.format(s[1]) + " " + '{:f}'.format(s[2]) + CEND)
                else:
                    positionCount += 1
                    f.write(str(stringOut).replace("'","")[1:-1] + "\n")
                    print('==> Singularity: {:f}'.format(s[0]) + " " + '{:f}'.format(s[1]) + " " + '{:f}'.format(s[2]))
            time.sleep(0.2)

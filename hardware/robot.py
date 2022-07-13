from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import numpy as np
import math
import socket
import time

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
                self.s.connect((self.config['host'], int(self.config['port'])))
                # RTDE connect
                self.rtde_c = RTDEControl(self.config['host'])
                self.rtde_r = RTDEReceive(self.config['host'])
                # Wait 0.05 seconds because the robot communication speed is 125 Hz = 0.008 seconds
                time.sleep(0.05)
                # Connected
                self.msg_connected()
                self.path = self.config['save_path']
            except socket.error as msg:
                self.connected = False
                print("Caught exception socket.error : %s" % msg)
            except TypeError as msg:
                self.connected = False
                print("Type Error: %s" % msg)


    def home(self):
        self.move([0,-1.57,0,-1.57,0,0])

    def move(self, position, wait=True):
        self.rtde_c.moveJ(position, 0.8, 0.8)
        # wait for move to finish
        if wait:
            while not self.rtde_c.isSteady():
                time.sleep(0.2)
        # pause before continuation
        time.sleep(1)

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

    def manual_positions(self):
        print("Record positions, enter to log position, q to quit.")
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
            print([ '%.5f' % elem for elem in self.rtde_r.getActualQ() ])

            time.sleep(0.2)

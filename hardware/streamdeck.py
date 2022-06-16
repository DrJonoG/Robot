from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import robot
import time
import sys

# Testing functions
def testSingleMove(r):
    r.movej("-1.48,-2.32,0.89,-1.86,-4.67,0.75",20)

# Demo functions

def randomMoves(r):
    rtde_c = RTDEControl("10.157.37.10")
    rtde_r = RTDEReceive("10.157.37.10")
    # Perform movements
    rtde_c.moveJ([-0.00, -1.57, 0.00, -1.56, 0.00, 0.00], 1.5, 1.8)
    for i in range(0, 3):
        rtde_c.moveJ([1.511, -1.570, -0.008, -1.5699, 0.001, 0.002], 1.5, 1.8)
        rtde_c.moveJ([1.687, -1.521, 1.384, 0.344, 0.2095, 1.572], 1.5, 1.8)
        rtde_c.moveJ([1.690, -1.521, 2.221, 0.211, 2.581, 1.572], 1.5, 1.8)
        rtde_c.moveJ([2.719, -1.522, 2.068, 0.211, 2.581, 1.572], 1.5, 1.8)
        rtde_c.moveJ([2.719, -1.595, 1.351, -2.726, 0.410, 1.592], 1.5, 1.8)
        rtde_c.moveJ([2.716, -2.606, 1.9881, 0.2651, -0.4239, 1.5993], 1.5, 1.8)
        rtde_c.moveJ([1.2544, -1.68734, 1.08590, 0.2648, -0.42199, 1.59926], 1.5, 1.8)

def demoNod(r):
    rtde_c = RTDEControl("10.157.37.10")
    rtde_r = RTDEReceive("10.157.37.10")
    # Perform movements
    rtde_c.moveJ([-0.00, -1.57, 0.00, -1.56, 0.00, 0.00], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.047, -1.836, -0.806, -1.441, 0.0003, 0.006], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.044, -3.279, -0.819, -1.4426, 0.0002, 0.007], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.047, -1.836, -0.806, -1.441, 0.0003, 0.006], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.044, -3.279, -0.819, -1.4426, 0.0002, 0.007], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.047, -1.836, -0.806, -1.441, 0.0003, 0.006], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.044, -3.279, -0.819, -1.4426, 0.0002, 0.007], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.0185, -1.898, -1.120, 0.460, 1.768, 0.002], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.0185, -1.8813, -1.1203, -1.155, 1.768, 0.002], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.0185, -1.898, -1.120, 0.460, 1.768, 0.002], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.0185, -1.8813, -1.1203, -1.155, 1.768, 0.002], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.0185, -1.898, -1.120, 0.460, 1.768, 0.002], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.0185, -1.8813, -1.1203, -1.155, 1.768, 0.002], 1.05, 1.4)
    time.sleep(0.2)


def demoWave(r):
    # Connect to control
    rtde_c = RTDEControl("10.157.37.10")
    rtde_r = RTDEReceive("10.157.37.10")
    # Perform movements
    rtde_c.moveJ([-0.00, -1.57, 0.00, -1.56, 0.00, 0.00], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.505, -1.569, 0.001, -1.569, 0.001, 0.003], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.451, -0.455, -1.155, -1.639, 0.0007, 0.545], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.451, -0.422, -0.518, -0.921, 0.0003, 0.4937], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.4507, -0.4304, -1.829, -2.4036, 0.001, 0.4937], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.451, -0.422, -0.518, -0.921, 0.0003, 0.493], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.450, -0.430, -1.829, -2.403, 0.001, 0.493], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.451, -0.422, -0.518, -0.921, 0.0003, 0.493], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.450, -0.430, -1.829, -2.403, 0.001, 0.493], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.451, -0.422, -0.518, -0.921, 0.0003, 0.493], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.450, -0.430, -1.829, -2.403, 0.001, 0.493], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.451, -0.422, -0.518, -0.921, 0.0003, 0.493], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.450, -0.430, -1.829, -2.403, 0.001, 0.493], 1.05, 1.4)
    time.sleep(0.2)

def demoRandom(r):
    # close gripper temporary hack
    f = open("C:/Users/jonat/OneDrive/Documents/Robot/hardware/close.script", "r")
    l = f.read(1024)

    while(l):
    	r.send(l)
    	l = f.read(1024)
    time.sleep(0.2)
    # Connect to control
    rtde_c = RTDEControl("10.157.37.10")
    rtde_r = RTDEReceive("10.157.37.10")
    # Perform movements
    rtde_c.moveJ([-0.00, -1.57, 0.00, -1.56, 0.00, 0.00], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([-0.013, -1.290, 2.049, -1.820, 0.178, -1.352], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([0.122, -0.945, 1.293, -0.678, 0.568, -0.391], 1.05, 1.4)
    time.sleep(0.2)
    # Open gripper temporary hack
    f = open("C:/Users/jonat/OneDrive/Documents/Robot/hardware/open.script", "r")
    l = f.read(1024)

    while(l):
    	r.send(l)
    	l = f.read(1024)
    time.sleep(0.2)
    # Need to re upload to avoid disconnecting
    rtde_c.reuploadScript()
    time.sleep(0.2)
    # continue
    rtde_c.moveJ([0.236, -0.847, 1.078, -0.408, 0.784, -0.612], 1.05, 1.4)
    time.sleep(0.2)

    # close gripper temporary hack
    f = open("C:/Users/jonat/OneDrive/Documents/Robot/hardware/close.script", "r")
    l = f.read(1024)

    while(l):
    	r.send(l)
    	l = f.read(1024)
    time.sleep(0.2)
    # Need to re upload to avoid disconnecting
    rtde_c.reuploadScript()
    time.sleep(0.2)
    # continue
    rtde_c.moveJ([1.774, -0.864, 1.029, -0.238, 1.346, -0.611], 1.05, 1.4)
    time.sleep(0.2)
    rtde_c.moveJ([1.850, 0.049, 1.046, 0.037, 1.536, 0.739], 1.05, 1.4)
    time.sleep(0.2)
    # Open gripper temporary hack
    f = open("C:/Users/jonat/OneDrive/Documents/Robot/hardware/open.script", "r")
    l = f.read(1024)

    while(l):
    	r.send(l)
    	l = f.read(1024)
    time.sleep(0.2)
    # Need to re upload to avoid disconnecting
    rtde_c.reuploadScript()
    time.sleep(0.2)

    # continue
    # Target in the Z-Axis of the TCP
    target = rtde_r.getActualTCPPose()



    # Stop the RTDE control script
    rtde_c.stopScript()

# Robot functions for usage

def robotCalibrate(r):
    raise NotImplementedError("has not been implemented")


if __name__ == '__main__':
    cmd = sys.argv[1]

    # Initialise robot and connect
    r = robot.UR()
    r.connect("10.157.37.10", 30003)

    # 'switch' statement to determine operation from streamdeck
    if cmd == "dMoves":
        randomMoves(r)
    elif cmd == "dNod":
        demoNod(r)
    elif cmd == "dWave":
        demoWave(r)
    elif cmd == "dRandom":
        demoRandom(r)
    elif cmd == "reset":
        r.home()
    elif cmd == "calibrate":
        robotCalibrate(r)
    elif cmd == "testMove":
        testSingleMove(r)
    else:
        # disconnect before raising error
        r.disconnect()
        raise NotImplementedError("Function has not been implemented")

    # disconnect
    r.disconnect()

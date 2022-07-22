from hardware import camera as C
from hardware import robot as R
from calibration import camera as CamCalib
from calibration import axyb as AXYB
from calibration import calibrate as Calib
from io_func import read_write as IO
from calibration import functions
from threading import Thread
import glob
import configparser
import numpy as np
import shutil

class main(object):
    def __init__(self, path, config):
        self.cam_thread = None
        self.robot_thread = None
        self.turntable_thread = None
        self.main_thread = None

        self.config = config
        self.config = configparser.ConfigParser()
        self.config.read("./config.ini")

        self.cam = C.fetchClass(self.config['hardware']['camera'], self.config)
        self.robot = R.fetchClass(self.config['hardware']['robot'], self.config['robot'])
        self.turntable = None
        self.calibration = Calib.Calibrate()

    def exit_app(self):
        # Exit all applications correctly.
        if not self.cam_thread == None:
            self.cam.stop_stream()
        if self.robot.connected == True:
            self.robot.disconnect()
        if self.cam.connected == True:
            self.cam.disconnect()

    def testf(self,tt,ttt):
        print(tt)

    def request_input(self):
        twt = "dfkhjjdfhfdhj"
        cmd = {
            'connect':[[self.cam.connect],[self.robot.connect]],
            'cam.connect':[[self.cam.connect]],
            'cam.live': [[self.cam.connect],[self.cam.livestream]],
            'cam.photo': [[self.cam.connect],[self.cam.capture]],
            'cam.capture': [[self.cam.connect],[self.cam.capture]],
            'test2': [[self.testf, twt, 'text2'],[self.testf, 'text3','text4']],
            'robot.connect':[[self.robot.connect]],
            'robot.open': [[self.robot.connect],[self.robot.open]],
            'robot.close': [[self.robot.connect],[self.robot.close]],
            'cam.calibrate': [[self.cam.connect],[self.robot.connect],[self.calibration, self.cam, self.robot, self.turntable, self.config]],
            'cam.calib': [[self.cam.connect],[self.robot.connect],[self.calibration, self.cam, self.robot, self.turntable, self.config]],
            'robot.calib': [[AXYB.run]],
            'axyb': [[AXYB.run]],
            'full': [[self.cam.connect],[self.robot.connect],[self.calibration, self.cam, self.robot, self.turntable, self.config], [AXYB.run]],
            'manual': [[self.cam.connect],[self.cam.livestream],[self.robot.connect],[self.robot.manual_positions]],
        }
        iter = True
        while iter:
            try:
                user_input = input('>> ')
                input_split = user_input.split(":")
                func = input_split[0]

                if func.lower() == "exit" or func.lower()  == "quit" or func.lower()  == "q":
                    self.exit_app()
                    exit()

                if type(cmd[func]) == list:
                    for k in range(0, len(cmd[func])):
                        vars = cmd[func][k][1:]
                        cmd[func][k][0](*vars)
            except Exception:
                print("==> Error: unsupported command")


    def start(self):
        self.main_thread = Thread(target=self.request_input, args=())
        self.main_thread.start()
        self.main_thread.join()


if __name__ == "__main__":
    # Start application
    interface = main("./", "./config.ini")
    interface.start()

from hardware import camera as C
from hardware import robot as R
from hardware import turntable as T
from calibration import camera as CamCalibration
from calibration import axyb as AXYB
from calibration import turntable as TTCalibration
from active import modelling as Model
from io_func import read_write as IO
from io_func import pmvs as PMVS
from io_func import filter as Filter
from calibration import functions
from threading import Thread
import glob
import configparser
import numpy as np
import shutil
import pathlib
import json
from consolemenu import *
from consolemenu.items import *
from consolemenu.format import *
from consolemenu.menu_component import Dimension
from consolemenu.prompt_utils import PromptUtils
import time

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
        self.robot = R.fetchClass(self.config['hardware']['robot'], self.config)
        self.turntable = T.fetchClass(self.config['hardware']['turntable'], self.config)
        self.camCalibration = CamCalibration.calibrateCamera(self.config['calibration']['working_dir'], self.config)
        self.ttCalibration = TTCalibration.calibrateTurntable(self.config)
        self.modelling = Model.ImageAcquisition()
        self.PMVS = PMVS.PMVS(self.config['model']['working_dir'])

        print("==> Working calibration directory: " + self.config['calibration']['working_dir'])
        print("==> Working model directory: " + self.config['model']['working_dir'])

    ## TODO: output help regarding menu here
    def printConfig(self):
        with open("./config.ini", mode="rt") as f:
            for line in f:
                print(line, end="")

    def exit_app(self):
        # Exit all applications correctly.
        if not self.cam_thread == None:
            self.cam.stop_stream()
        if self.robot.connected == True:
            self.robot.disconnect()
        if self.cam.connected == True:
            self.cam.disconnect()

    def request_input(self):
        cmd = {
            'connect.all':[[self.cam.connect],[self.robot.connect]],
            'cam.connect':[[self.cam.connect]],
            'cam.live': [[self.cam.connect],[self.cam.livestream]],
            'cam.photo': [[self.cam.connect],[self.cam.capture]],
            'cam.capture': [[self.cam.connect],[self.cam.capture]],
            'turntable.home': [[self.turntable.connect], [self.turntable.Home]],
            'turntable.connect': [[self.turntable.connect]],
            'turntable.goto': [[self.turntable.connect], [self.turntable.GoTo, 30]],
            'robot.connect':[[self.robot.connect]],
            'robot.home':[[self.robot.connect],[self.robot.home]],
            'robot.open': [[self.robot.connect],[self.robot.open]],
            'robot.close': [[self.robot.connect],[self.robot.close]],
            'calib.turntable': [
                [self.cam.connect],
                #[self.cam.livestream],
                [self.robot.connect],
                [self.turntable.connect],
                [self.ttCalibration.initialise, self.robot, self.cam, self.turntable],
                [self.ttCalibration.calibrate],
                [self.ttCalibration.estimateCenter],
                [self.camCalibration.averageIntrinsics],
            ],
            'calib.camera': [
                [self.cam.connect],
                [self.cam.livestream],
                [self.robot.connect],
                [self.camCalibration.capture, self.robot, self.cam, self.turntable],
                [self.camCalibration.calibrate]
            ],
            'robot.sing': [
                [self.robot.connect],
                [self.robot.singularity]
            ],
            'home.robot': [
                [self.robot.connect],
                [self.robot.home],
            ],
            'calib': [
                [self.cam.connect],
                [self.robot.connect],
                [self.turntable.connect],
                [self.ttCalibration.initialise, self.robot, self.cam, self.turntable],
                [self.camCalibration.capture, self.robot, self.cam, self.turntable],
                [self.camCalibration.calibrate],
                [AXYB.run],
                [self.camCalibration.estimateA,  self.config['calibration']['working_dir'] + "\\images\\", self.config['calibration']['working_dir'] + "\\axyb\\", self.config['calibration']['working_dir'] + "\\projected\\"],
                [self.exit_app],
            ],
            'calib.full': [
                [self.cam.connect],
                #[self.cam.livestream],
                [self.robot.connect],
                [self.turntable.connect],
                [self.ttCalibration.initialise, self.robot, self.cam, self.turntable],
                [self.ttCalibration.calibrate],
                [self.ttCalibration.estimateCenter],
                [self.camCalibration.capture, self.robot, self.cam, self.turntable],
                [self.camCalibration.calibrate],
                [AXYB.run],
                [self.camCalibration.estimateA,  self.config['calibration']['working_dir'] + "\\images\\", self.config['calibration']['working_dir'] + "\\axyb\\", self.config['calibration']['working_dir'] + "\\projected\\"],
                [self.exit_app],
            ],
            'robot.move' : [
                [self.robot.connect],
                [self.robot.move, np.array([4.37671, -3.72725, 1.33090, -2.14083, 2.15675, -0.67304])]
            ],
            'record.position': [
                [self.cam.connect],
                [self.cam.livestream],
                [self.robot.connect],
                [self.robot.manual_positions]
            ],
            'model.recon': [
                [self.PMVS.createFiles],
                [self.PMVS.run],
                [Filter.filterPoints,  self.config['model']['working_dir'] + r'\models\option.txt.ply', self.config['model']['working_dir'] + r'\models\filtered.ply', self.config]
            ],
            'image': [
                [self.cam.connect],
                [self.robot.connect],
                [self.turntable.connect],
                [self.modelling.initialise, self.robot, self.cam, self.turntable, self.config],
                [self.modelling.run]
            ],
            'model.full': [
                [self.cam.connect],
                #[self.cam.livestream],
                [self.robot.connect],
                [self.turntable.connect],
                [self.modelling.initialise, self.robot, self.cam, self.turntable, self.config],
                [self.modelling.run],
                [self.exit_app],
                [self.PMVS.createFiles],
                [self.PMVS.run],
                [Filter.filterPoints,  self.config['model']['working_dir'] + r'\models\option.txt.ply', self.config['model']['working_dir'] + r'\models\filtered.ply', self.config]
            ],
            'config': [
                [self.printConfig]
            ]
        }
        iter = True

        # Output command list
        print("==> Please use one of the following commands: ")
        print('\n'.join("    " + str(key) for key, value in dict(sorted(cmd.items())).items()))

        # Await input
        while iter:
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


    def start(self):
        self.main_thread = Thread(target=self.request_input, args=())
        self.main_thread.start()
        self.main_thread.join()


if __name__ == "__main__":
    # Start application
    interface = main("./", "./config.ini")
    interface.start()

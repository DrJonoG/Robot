from hardware import camera, robot, turntable
from calibration import calibration_axyb, calibration_camera, calibration_functions
from threading import Thread
import configparser

class main_interface(object):
    def __init__(self, path, config):
        self.cam_thread = None
        self.robot_thread = None
        self.turntable_thread = None
        self.main_thread = None
        self.config = config

        self.cam = None
        self.save_path = path

    def exit(self):
        iter = False

    def cam_calibration(self):
        c = self.config['cam_calibration']
        self.cam_calibration = cameraCalibration.calibrateCamera(c['input_path'], c['output_path'], c['cb_length'], c['cb_width'], c['cb_size'])
        self.cam_calibration.calibrate()

    def cam_threaded(self):
        self.cam_thread = self.cam.livestream()
        self.cam_thread.start()

    def request_input(self):
        #cam.connect(0)
        #cam.livestream()
        cmd = {'cam.connect':self.cam.connect, 'cam.live': self.cam_threaded, 'cam.photo': self.cam.takephoto, 'exit':exit, 'cam.calibrate': self.cam_calibration }
        iter = True
        while iter:
            #try:
            user_input = input('>> ')
            input_split = user_input.split(" ")
            func = input_split[0]

            if func.lower() == "exit" or func.lower()  == "quit" or func.lower()  == "q":
                if not self.cam_thread == None:
                    self.cam.stop_stream()
                break
            elif len(input_split) > 1:
                vars = input_split[1:]
                cmd[func](vars)
            else:
                cmd[func]()
            #except:
            #    print("==> Error: Invalid function, please use 'help' for function list")

    def run(self):
        cam = {'Logi': camera.Logi, 'Canon': camera.Canon }
        try:
            self.cam = cam[self.config['hardware']['camera']]()
        except:
            print("==> Error: Unsupported camera format. Please use 'help'")
            return

        self.main_thread = Thread(target=self.request_input, args=())
        self.main_thread.start()
        self.main_thread.join()

        #directory = "C:/Users/User/Documents/01.PythonProjects/robot/data/sweetcorn/"
        #axyb = axybCalibration.AXYB(directory)
        #axyb.load()
        #axyb.dornaika()

from hardware import camera as C
from hardware import robot as R
from calibration import calibration_camera as CamCalib
from calibration import calibration_axyb as AXYB
from io_func import read_write as IO
from calibration import calibration_functions
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


    def calibration(self):
        # Reset to home positon
        self.robot.home()

        # Temporary manual positions
        robotPositions = [
            [-1.5910828749286097, -3.4696413479247035, 1.1608193556415003, -1.7051917515196742, -4.600187842045919, 0.8421457409858704],
            [-1.9187200705157679, -3.5524121723570765, 0.9695852438556116, -1.4929271799376984, -4.157844845448629, 0.21954363584518433],
            [-1.7760065237628382, -3.1099077663817347, 0.9322660605060022, -1.869436880151266, -4.3982089201556605, 0.506886899471283],
            [-1.4750688711749476, -3.110176225701803, 0.5018327871905726, -1.4722391825965424, -4.681981388722555, 0.7737915515899658],
            [-1.614232365285055, -3.1101461849608363, 0.5018633047686976, -1.5715543232359828, -4.471733752881185, 0.31390905380249023],
            [-1.1875951925860804, -3.256608625451559, 1.1464913527118128, -1.930387636224264, -5.028140846882955, 1.0112882852554321],
            [-0.8962963263141077, -3.4728619060912074, 1.1411212126361292, -1.8624440632262171, -5.313315276299612, 1.5295062065124512],
            [-0.7660658995257776, -3.623277326623434, 1.14042836824526, -1.7382713756956996, -5.427960220967428, 1.5915594100952148],
            [-1.0399377981769007, -3.3578230343260707, 1.1188300291644495, -1.8541728458800257, -5.134522859250204, 1.4392145872116089],
            [-1.078259293233053, -2.903804441491598, 1.1190345923053187, -2.2189070187010707, -4.988383475934164, 1.032646656036377],
            [-0.9558184782611292, -3.362876077691549, 0.5334718863116663, -1.358720527296402, -5.2308810392962855, 1.4002090692520142],
            [-0.774571720753805, -3.2981025181212367, 1.0364635626422327, -2.0157276592650355, -5.3614113370524805, 1.6982053518295288]
        ]

        # Go to each position and capture images
        for i in range(0, len(robotPositions)):
            # Move robot to position
            self.robot.move(robotPositions[i])
            # Capture image
            currentFile = self.cam.capture()
            # Calculate robot transformation matrix
            robotMatrix = self.robot.angles_to_transformation()
            # Write matrix
            IO.wMatrix(robotMatrix, currentFile, self.config['calibration']['working_dir'] + '/axyb/', 'B')

        # Initialise calibration and parameters
        camCalib = CamCalib.calibrateCamera(self.config['calibration']['working_dir'] + '/images/', self.config['calibration']['out_path'], self.config['calibration']['cb_width'], self.config['calibration']['cb_length'], self.config['calibration']['cb_size'])

        # Perform calibration
        camCalib.calibrate()

        # Project point to 0 for verification
        #for filepath in glob.iglob(self.config['calibration']['working_dir'] + '\images\*.jpg'):
        #    imagePath = filepath
        #    outputPath = filepath.replace("images","projected")
        #    txtPath = filepath.replace("images","projection").replace(".jpg",".txt")
        #    # Project point
        #    self.cam.projectPoint(np.array([0,0,0,1]), txtPath, imagePath, outputPath)


    def exit_app(self):
        # Exit all applications correctly.
        if not self.cam_thread == None:
            self.cam.stop_stream()
        if self.robot.connected == True:
            self.robot.disconnect()
        if self.cam.connected == True:
            self.cam.disconnect()

    def testf(self, textD, textdd):
        print(textD)
        print(textdd)

    def request_input(self):
        cmd = {
            'connect':[self.cam.connect,self.robot.connect],
            'cam.connect':self.cam.connect,
            'cam.live': [self.cam.connect,self.cam.livestream],
            'cam.photo': [self.cam.connect,self.cam.capture],
            'cam.capture': [self.cam.connect,self.cam.capture],
            'test': [[self.testf, 'text']],
            'test2': [[self.testf, 'text', 'text2'],[self.testf, 'text3','text4','text54']],
            'robot.connect':self.robot.connect,
            'robot.open': [self.robot.connect,self.robot.open],
            'robot.close': [self.robot.connect,self.robot.close],
            'cam.calibrate': [self.cam.connect,self.robot.connect,self.calibration],
            'cam.calib': [self.cam.connect,self.robot.connect,self.calibration],
            'robot.calib':AXYB.run,
            'axyb':AXYB.run,
            'full': [self.cam.connect,self.robot.connect,self.calibration, AXYB.run],
            'manual': [self.cam.connect,self.cam.livestream,self.robot.connect,self.robot.manual_positions],
        }
        iter = True
        while iter:
            #try:
            user_input = input('>> ')
            input_split = user_input.split(":")
            func = input_split[0]

            if func.lower() == "exit" or func.lower()  == "quit" or func.lower()  == "q":
                self.exit_app()
                break
            elif len(input_split) > 1:
                vars = input_split[1:]
                cmd[func](vars)/00000000000
            else:
                if type(cmd[func]) == list:
                    for i in range(0, len(cmd[func])):
                        cmd[func][i]()
                else:
                    cmd[func]()
            #except:
            #    print("==> Error: Invalid function, please use 'help' for function list")
            #for i in range(0, len(cmd[func])):
            #    print(cmd[func][i][1:])
                #cmd[func][i][0](cmd[func][i][1:])
            #elif len(input_split) > 1:
            #    vars = input_split[1:]
            #    cmd[func](vars)/00000000000
            #else:
            #    if type(cmd[func]) == list:
            #        for i in range(0, len(cmd[func])):
            #            cmd[func][i]()
            #    else:
            #        cmd[func]()
            #except:
            #    print("==> Error: Invalid function, please use 'help' for function list")

    def start(self):
        self.main_thread = Thread(target=self.request_input, args=())
        self.main_thread.start()
        self.main_thread.join()


if __name__ == "__main__":
    # Start application
    interface = main("./", "./config.ini")
    interface.start()

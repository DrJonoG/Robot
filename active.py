import configparser
import numpy as np
import glob
from hardware import robot as R
from hardware import camera as C
from calibration import calibration_camera as CamCalib
from io_func import read_write as IO

if __name__ == "__main__":
    # Process config file here
    config = configparser.ConfigParser()
    config.read("./config.ini")

    # Fetch camera class
    cam = C.fetchClass(config['hardware']['camera'])
    cam.connect(config['cam_calibration']['path'])

    for filepath in glob.iglob(config['cam_calibration']['out_path'] + 'images\*.jpg'):
        print("==> Processing " + filepath)
        imagePath = filepath
        outPath = filepath.replace("images","projected")
        txtPath = filepath.replace("images","projection").replace(".jpg",".txt")

        cam.projectPoint(np.array([0,0,0,1]), txtPath, imagePath, outPath)


    # disconnect
    cam.disconnect()

import subprocess
import datetime


from os import system, getcwd, makedirs

def func_TakeNikonPicture():
    #system('"C:\\Program Files (x86)\\digiCamControl\\cameracontrolcmd.exe" /filename C:\\Users\\jonat\\Pictures\\digiCamControl\\test.jpg /capturenoaf /wait 200')
    digiCamPath = "C:\\Program Files (x86)\\digiCamControl\\cameracontrolcmd.exe"
    savePath = "C:\\Users\\jonat\\OneDrive - The University of Nottingham\\GitHub\\Robot\\data\\calibration\\images\\test.jpg"
    subprocess.call(f'"{digiCamPath}" /filename "{savePath}" /capturenoaf /wait 200', shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    #This makes the wait possible


    print('done')

#from DigiCam.Camera import Camera

# Replace the below path with the absolute or relative path to your CameraControlCmd executable.
#camera_control_cmd_path = 'C:\\Program Files (x86)\\digiCamControl\\CameraControlCmd.exe'

#test_camera = Camera(control_cmd_location=camera_control_cmd_path)

#test_camera.capture_single_image(autofocus=False)

# Store date/time for file uniqueness


# take picture
func_TakeNikonPicture()

import os
import subprocess
import ctypes, sys
import cv2
import numpy as np

class PMVS(object):
    def __init__(self, coreDir):
        # pmvs default paths
        self.coreDir = coreDir
        self.imageDir = coreDir + "/visualize/"
        self.txtDir = coreDir + "/txt/"

    def prepend_line(self, file_name, line):
        """ Insert given string as a new line at the beginning of a file """
        # define name of temporary dummy file
        dummy_file = file_name + '.bak'
        # open original file in read mode and dummy file in write mode
        with open(file_name, 'r') as read_obj, open(dummy_file, 'w') as write_obj:
            # Write given line to the dummy file
            write_obj.write(line + '\n')
            # Read lines from original file one by one and append them to the dummy file
            for line in read_obj:
                write_obj.write(line)
        # remove original file
        os.remove(file_name)
        # Rename dummy file as the original file
        os.rename(dummy_file, file_name)

    # Filter image backgrounds
    def filterBackground(self, dir):
        for file in os.listdir(dir):
            filename = os.fsdecode(file)
            if filename.endswith( ('.jpeg', '.png', '.gif', '.jpg') ):
                # load image
                img = cv2.imread(dir + filename)
                #convert the BGR image to HSV colour space
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                #set the lower and upper bounds for the green hue
                lower_green = np.array([0,0,0])
                upper_green = np.array([255,255,150])
                #create a mask for green colour using inRange function
                mask = cv2.inRange(hsv, lower_green, upper_green)
                #perform bitwise and on the original image arrays using the mask
                res = cv2.bitwise_and(img, img, mask=mask)
                # Output
                cv2.imwrite(dir + filename, res)

    # Helper file to prepare folders and files for use with PMVS
    def createFiles(self):
        # Filter images
        #self.filterBackground(self.imageDir)
        # Process
        imgCount = len(os.listdir(self.imageDir))
        # Create option file
        with open(self.coreDir + '/option.txt', 'w+') as f:
            f.write(f"level 0 csize 1 threshold 0.7 wsize 20 minImageNum 5 CPU 30 useVisData 0 sequence -1 timages -1 0 {imgCount} oimages 0")

        # Check top line is 'CONTOUR'
        for filename in os.listdir(self.txtDir):
            line = ""
            with open(os.path.join(self.txtDir, filename)) as f:
                line = f.readline()
            if 'contour' not in line.lower():
                self.prepend_line(self.txtDir + filename, "CONTOUR")

        # Create vis file
        print("==> PMVS core files created")

    def run(self):
        print("==> Running PMVS")
        #ctypes.windll.shell32.ShellExecuteW(None, ".\PMVS\pmvs2.exe", sys.executable, " ".join(sys.argv), None, 1)
        if not os.path.exists(self.coreDir + "/models/"):
            os.mkdir(self.coreDir + "/models/")
        subprocess.call(['.\PMVS\pmvs2.exe', self.coreDir, 'option.txt'], shell=True)
        #os.system(f"C:/Users/jonat/OneDrive/Documents/Robot/PMVS/pmvs2.exe {coreDir} option.txt")

    def cropOutput(self):
        pass

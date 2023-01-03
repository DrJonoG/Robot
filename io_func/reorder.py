import os
import shutil



for j in range(0, 7):
    textFolder = "I:\\CalibrationTurntable\\turntable\\" + str(j) + "\\axyb\\"
    imageFolder = "I:\\CalibrationTurntable\\turntable\\" + str(j) + "\\images\\"
    destination = "I:\\CalibrationTurntable\\turntable\\" + str(j) + "\\axyb\\"
    counter = 0
    for filename in os.listdir(imageFolder):
        os.rename(imageFolder + filename, imageFolder + str(counter).rjust(8, '0') + ".jpg")
        shutil.move(textFolder + "A" + filename.replace(".jpg",".txt"), destination + "A" + str(counter).rjust(8, '0') + ".txt")
        counter = counter + 1

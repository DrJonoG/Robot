import os
import shutil
import glob
from pathlib import Path
import re
from datetime import datetime

counter = 0

# # TEMP:
errorList = [
    'I:\\CalibrationFiles\\Robot\\images\\00000000.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000007.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000023.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000027.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000028.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000033.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000070.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000071.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000072.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000093.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000095.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000096.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000098.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000100.jpg',
    'I:\\CalibrationFiles\\Robot\\images\\00000101.jpg',

]

outputDirectory = "I:\\CalibrationFiles\\Robot\\"
# Remove all error files in errorList and append directories to be re-ordered
foldersToReOrder = []
for image in errorList:
    fileName = Path(image).stem
    allFiles = glob.glob(outputDirectory + "*\\*" + fileName + '.*', recursive=True)
    for file in allFiles:
        print(datetime.now().strftime('%H:%M:%S') + " ==> Deleting file " + str(file))
        os.remove(file)
        if os.path.dirname(file) not in foldersToReOrder:
            foldersToReOrder.append(os.path.dirname(file))

# # TEMP:

foldersToReOrder = [
    'I:\\CalibrationFiles\\Robot\\axyb',
    'I:\\CalibrationFiles\\Robot\\images',
    'I:\\CalibrationFiles\\Robot\\projected',
    'I:\\CalibrationFiles\\Robot\\projection'
]
# Re order directories
x = re.compile("[0-9]")
for folder in foldersToReOrder:
    print(datetime.now().strftime('%H:%M:%S') + " ==> Updating folder " + str(folder))
    counters = 0
    characters = ""
    for file in glob.glob(folder + "\\*.*"):
        # Get file name
        fileName = Path(file).stem
        # Check if current file is numbered, if not ignore
        numbered = len(re.findall(x, fileName))
        # If less than 7, do not renumber
        if numbered < 8:
            continue
        # Check for extra text
        extraCharacters = len(fileName) - 8
        if extraCharacters > 0:
            if fileName[0:extraCharacters] == characters:
                os.rename(file, folder + "\\" + characters + str(counters).rjust(8, '0') + Path(file).suffix)
                counters = counters + 1
            else:
                # Reset for new character
                characters = fileName[0:extraCharacters]
                counters = 0
                # Begin assignment
                os.rename(file, folder + "\\" + characters + str(counters).rjust(8, '0') + Path(file).suffix)
                # Update for next increment
                counters += 1
        else:
            os.rename(file, folder + "\\" + str(counters).rjust(8, '0') + Path(file).suffix)
            counters = counters + 1

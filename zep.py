import glob
import os
import numpy as np

with open(r'I:\Data\Plant1\zep.txt', 'w') as f:
    for filepath in glob.iglob(r'I:\Data\Plant1\A\*.txt'):
        outputString = os.path.basename(filepath).replace(".txt",".jpg")
        matrix = np.loadtxt(filepath)
        outputString = outputString + " " + str(matrix[0,3]) + " " + str(matrix[1,3]) + " " + str(matrix[2,3])
        outputString = outputString + " " + str(matrix[0,0]) + " " + str(matrix[0,1]) + " " + str(matrix[0,2])
        outputString = outputString + " " + str(matrix[1,0]) + " " + str(matrix[1,1]) + " " + str(matrix[1,2])
        outputString = outputString + " " + str(matrix[2,0]) + " " + str(matrix[2,1]) + " " + str(matrix[2,2]) + "\n"
        f.write(outputString)

with open(r'I:\Data\Plant1\zep_internal.txt', 'w') as f:
    for filepath in glob.iglob(r'I:\Data\Plant1\A\*.txt'):
        outputString = os.path.basename(filepath).replace(".txt",".jpg")
        outputString = outputString + " 2279.34731927 2279.58101507 1934.44886355 1113.44688863 0 4.40123531 -14.54102001 -134.28258960 0.00936685 -0.00104708" + "\n"
        f.write(outputString)

print("Complete")

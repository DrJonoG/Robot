import numpy as np
from calibration import functions as functions
import os
cls = lambda: os.system('cls')
cls()

def displayError(original, estimate, title, disp=False):
    err = original-estimate

    if disp:
        print("---------------- " + title + " ----------------")
        print(err)
        rotationError = np.round(np.sum(np.absolute(err[0:3, 0:3])), 5)
        translationError = np.round(np.sum(np.absolute(err[0:3, 3])), 5)
        print("--> Rotation error: " + str(rotationError))
        print("--> Translation error: " + str(translationError) + "\n\n")

    return err

np.set_printoptions(suppress=True)
axybPath = 'I:\\DeleteInApr\\\CalibrationFinal\\axyb\\'
#axybPath = 'I:\\CalibrationFiles\\RobotCurrent\\axyb\\'

XFile = axybPath + 'X.txt'
YFile = axybPath + 'Y.txt'
X = np.loadtxt(XFile)
Y = np.loadtxt(YFile)

totalFiles = 0
totalAError = None
totalYError = None
totalAEstError = None
for i in range(0, 300):
    file = str(i)
    file = file.zfill(8)

    AFile = axybPath + 'A' + file + '.txt'
    BFile = axybPath + 'B' + file + '.txt'

    if not os.path.exists(axybPath + 'A' + file + '.txt'):
        break

    A = np.loadtxt(AFile)
    B = np.loadtxt(BFile)

    # Estimate A
    AEst = np.dot(X, functions.matrix_inverse(np.dot(Y, B)))
    dovA = np.dot(np.dot(Y,B) ,functions.matrix_inverse(X))
    err = displayError(A, AEst, 'Estimate A')
    err = displayError(A, dovA, 'Estimate Dov A')
    if i == 0:
        totalAError = err
    else:
        totalAError = totalAError + err

    # Estimate Y
    YEst = np.dot(functions.matrix_inverse(B), np.dot(X, A))
    YEst = functions.matrix_inverse(YEst)

    dovY = np.dot(A,X)
    dovY = np.dot(dovY,functions.matrix_inverse(B))
    err = displayError(Y, YEst, 'Estimate Y', True)
    err = displayError(Y, dovY, 'Estimate Dov Y', True)
    if i == 0:
        totalYError = err
    else:
        totalYError = totalYError + err
    exit()
    # Estimate A using estimate of Y
    AEstY = np.dot(X, functions.matrix_inverse(np.dot(YEst, B)))
    err = displayError(A, AEstY, 'Estimate A using Estimate Y')
    if i == 0:
        totalAEstError = err
    else:
        totalAEstError = totalAEstError + err

    # Adjust count
    totalFiles += 1

print("========> Total A Estimate Errors")
totalAError = (totalAError / totalFiles)
rotationError = np.round(np.sum(np.absolute(totalAError[0:3, 0:3])), 5)
translationError = np.round(np.sum(np.absolute(totalAError[0:3, 3])), 5)
print("--> Rotation error: " + str(rotationError))
print("--> Translation error: " + str(translationError) + "\n\n")

exit()






















print("========> Total Y Estimate Errors")
totalYError = (totalYError / totalFiles)
rotationError = np.round(np.sum(np.absolute(totalYError[0:3, 0:3])), 5)
translationError = np.round(np.sum(np.absolute(totalYError[0:3, 3])), 5)
print("--> Rotation error: " + str(rotationError))
print("--> Translation error: " + str(translationError) + "\n\n")

print("========> Total A Estimate Errors Using Y Estimate")
totalAEstError = (totalAEstError / totalFiles)
rotationError = np.round(np.sum(np.absolute(totalAEstError[0:3, 0:3])), 5)
translationError = np.round(np.sum(np.absolute(totalAEstError[0:3, 3])), 5)
print("--> Rotation error: " + str(rotationError))
print("--> Translation error: " + str(translationError) + "\n\n")




AFile = axybPath + 'A00000000.txt'
BFile = axybPath + 'B00000000.txt'
XFile = axybPath + 'X.txt'
YFile = axybPath + 'Y.txt'


A = np.loadtxt(AFile)
B = np.loadtxt(BFile)
X = np.loadtxt(XFile)
Y = np.loadtxt(YFile)

# Calculate estimate of A
AEst = np.dot(X, functions.matrix_inverse(np.dot(Y, B)))
err = A-AEst

print("---------------- A Estimate ----------------")
print(err)
rotationError = np.round(np.sum(np.absolute(err[0:3, 0:3])), 5)
translationError = np.round(np.sum(np.absolute(err[0:3, 3])), 5)
print("--> Rotation error: " + str(rotationError))
print("--> Translation error: " + str(translationError) + "\n\n")

# Calculate estimate of A
YEst = np.dot(functions.matrix_inverse(B), np.dot(X, A))
YEst = functions.matrix_inverse(YEst)

print("---------------- Y Estimate ----------------")
err = Y-YEst
print(err)
rotationError = np.round(np.sum(np.absolute(err[0:3, 0:3])), 5)
translationError = np.round(np.sum(np.absolute(err[0:3, 3])), 5)
print("--> Rotation error: " + str(rotationError))
print("--> Translation error: " + str(translationError) + "\n\n")


# Verify with A
AEstY = np.dot(X, functions.matrix_inverse(np.dot(YEst, B)))
err = A-AEstY

print("---------------- A with YEst Verficiation ----------------")
print(err)
rotationError = np.round(np.sum(np.absolute(err[0:3, 0:3])), 5)
translationError = np.round(np.sum(np.absolute(err[0:3, 3])), 5)
print("--> Rotation error: " + str(rotationError))
print("--> Translation error: " + str(translationError) + "\n\n")

print("---------------- A vs A with YEst Verficiation ----------------")
err = AEst-AEstY
print(err)
rotationError = np.round(np.sum(np.absolute(err[0:3, 0:3])), 5)
translationError = np.round(np.sum(np.absolute(err[0:3, 3])), 5)
print("--> Rotation error: " + str(rotationError))
print("--> Translation error: " + str(translationError) + "\n\n")

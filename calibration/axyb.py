"""
Calibration of the robot (UR5)
Created by Jonathon Gibbs, please reference if used.

www.jonathongibbs.co.uk
https://github.com/pszjg
"""
from calibration import functions as functions
import configparser
import numpy as np
import glob
import os
import glob
import re


def estimateY(config):
    np.set_printoptions(suppress=True)
    path = config['calibration']['tt']
    totalX = []
    for X in glob.glob(path + "*\\*\\X.txt",recursive = True):
            totalX.append(np.loadtxt(X))

    print(np.asarray(totalX))
    print("---------------")
    print(np.mean(totalX, axis=0))

class AXYB(object):
    def __init__(self, config, source):
        np.set_printoptions(suppress=True)
        # axyb path
        if source:
            self.working_dir = source
            self.txt_path = source + '\\axyb\\'
        else:
            self.working_dir = config['model']['working_dir']
            self.txt_path = config['calibration']['working_dir'] + '\\axyb\\'
        # Check if path exists
        if not os.path.exists(self.txt_path):
            print("==> Error, calibration path '" + self.txt_path + "' does not exist")
            return False

        self.files = 0
        # Declare empty variables
        (self.A, self.B, self.X, self.Y) = [],[],[],[]

    def load(self):
        # Obtain and sort files
        a_files = glob.glob(self.txt_path + "A*.txt")
        a_files.sort(key=lambda var:[int(x) if x.isdigit() else x for x in re.findall(r'[^0-9]|[0-9]+', var)])

        b_files = glob.glob(self.txt_path + "B*.txt")
        b_files.sort(key=lambda var:[int(x) if x.isdigit() else x for x in re.findall(r'[^0-9]|[0-9]+', var)])

        # Check equal files have been found
        if len(a_files) != len(b_files):
            print("==> Error loading calibration files. There are not and equal number of A and B files.")
            exit()

        # Number of files
        self.files = len(a_files)
        # Load files into matrices
        for i in range(self.files):
            self.A.append(functions.matrix_inverse(np.loadtxt(a_files[i])))
            self.B.append(np.loadtxt(b_files[i]))


    def dornaika(self):
        # Calculate the rotation matrix
        C = np.zeros(shape=(4,4))
        for i in range(self.files):
            a_quat = functions.rot2q(self.A[i][0:3, 0:3])
            b_quat = functions.rot2q(self.B[i][0:3, 0:3])
            QA = functions.skewqA(a_quat)
            WB = functions.skewqB(b_quat)
            C = C - np.dot(np.linalg.inv(QA), WB)

        u, s, v = np.linalg.svd(C)
        v = np.transpose(v)


        (x, y) = np.zeros(shape=(4)), np.zeros(shape=(4))
        (x_rot, y_rot) = np.zeros(shape=(3,3)), np.zeros(shape=(3,3))

        ####
        # New
        ####

        diagonal = np.zeros(shape=(s.shape[0],s.shape[0]))
        np.fill_diagonal(diagonal, s)
        s = self.files - s
        # Minimum index of s
        diagonal_s = np.argmin(s)

        # Assign x and y
        x = np.concatenate((u[1:4, diagonal_s], u[0,diagonal_s]), axis=None)
        y = np.concatenate((v[1:4, diagonal_s], v[0,diagonal_s]), axis=None)


        M_x = functions.q2rot(x)
        M_y = functions.q2rot(y)

        # Calculate the translation
        a_tran = np.zeros(shape=(self.files*3, 6))
        b_tran = np.zeros(shape=(self.files*3))
        identity = np.identity(3)

        for i in range(self.files):
            neg_R = -self.A[i][0:3, 0:3]
            k = np.concatenate((neg_R, identity), axis=1)

            a_tran[i * 3] = k[0,:6]
            a_tran[i * 3 + 1] = k[1,:6]
            a_tran[i * 3 + 2] = k[2,:6]

            AT = self.A[i][0:3,3]
            diagonal_T = np.zeros(shape=(3,3))
            np.fill_diagonal(diagonal_T, self.B[i][0:3,3])
            solve = np.dot(np.kron(self.B[i][0:3,3].flatten(), identity), M_y.flatten())
            solve_T = AT - solve

            b_tran[i * 3] = solve_T[0]
            b_tran[i * 3 + 1] = solve_T[1]
            b_tran[i * 3 + 2] = solve_T[2]

        T_vec, _, _, _ = np.linalg.lstsq(a_tran,b_tran, rcond=-1)

        xRT, yRT = np.identity(4), np.identity(4)
        xRT[0:3, 0:3] = M_x
        xRT[0:3, 3] = T_vec[0:3]

        yRT[0:3, 0:3] = M_y
        yRT[0:3, 3] = T_vec[3:6]

        np.savetxt(self.txt_path + "X.txt", xRT, fmt='%.8f')
        np.savetxt(self.txt_path + "Y.txt", yRT, fmt='%.8f')

        print("==> Calibration saved to " + self.txt_path)
        print("==> AXYB has successfully been calculated")


    def verifyEstimation(self):
        X = self.txt_path + "\X.txt"
        Y = self.txt_path + "\Y.txt"

        if not os.path.exists(X) or not os.path.exists(Y):
            print("==> Error: Unable to verify estimate. Missing calibration files.")
            return

        X = np.loadtxt(X)
        Y = np.loadtxt(Y)
        # Variables
        rError = np.empty([3,3])
        tError = np.zeros([3])
        tErrorAll = np.array([])
        rErrorAll = np.array([])
        counter = 0
        for file in os.listdir(self.txt_path):
            if file.startswith("A0") and os.path.exists(self.txt_path + file.replace("A0","B0")):
                AValidation = self.txt_path + "\\" + file
                B = self.txt_path + "\\" + file.replace("A0","B0")

                # Load paths
                AValidation = np.loadtxt(AValidation)
                B = np.loadtxt(B)

                # Calculate estimate of A
                A = np.dot(Y, B)
                A = np.dot(X, functions.matrix_inverse(A))

                counter = counter + 1

                tError = tError + (AValidation[0:3, 3] - A[0:3, 3])
                rError = rError + (AValidation[0:3, 0:3] - A[0:3, 0:3])

                tErrorAll = np.append(tErrorAll,[(AValidation[0:3, 3] - A[0:3, 3])])
                rErrorAll = np.append(rError,[(AValidation[0:3, 0:3] - A[0:3, 0:3])])


        if counter > 0:
            print("==> Following values should be 0 to show estimation is be calculated the same")


            #print(tErrorAll.reshape(-1,3))

            rotationError = np.round((rError) / counter, 5)
            totalError = np.round(np.sum(np.absolute(rotationError)), 5)
            print("Rotation Error Total: "  + str(totalError) + "\n" + str(rotationError) + "")
            np.savetxt(self.txt_path + "\\rError.txt", rotationError, fmt='%.6f')

            translationError = np.round(tError / counter, 5)
            totalError = np.round(np.sum(np.absolute(translationError)), 5)
            print("Translation Error Total: " + str(totalError) + "\n" + str(translationError) + "")
            np.savetxt(self.txt_path + "\\tError.txt", translationError, fmt='%.6f')


def run(source=None):
    config = configparser.ConfigParser()
    config.read("./config.ini")
    # Initialise
    axyb = AXYB(config, source)
    # Load files
    axyb.load()
    # Calculate dornaika
    axyb.dornaika()
    # Estimate error
    axyb.verifyEstimation()

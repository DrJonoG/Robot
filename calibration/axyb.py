"""
Calibration of the robot (UR5)
Created by Jonathon Gibbs, please reference if used.

www.jonathongibbs.co.uk
https://github.com/pszjg
"""
from calibration import functions as functions
import configparser
import numpy as np
import os
import glob
import re

class AXYB(object):
    def __init__(self, config):
        np.set_printoptions(suppress=True)
        # Check if path exists
        if not os.path.exists(config['calibration']['working_dir'] + '\\axyb\\'):
            print("==> Error, calibration path '" + config['calibration']['working_dir'] + "\\axyb\\' does not exist")
            return False

        self.txt_path = config['calibration']['working_dir'] + '\\axyb\\'
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
            return None

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

        diagonal = np.zeros(shape=(4,4))
        np.fill_diagonal(diagonal, s)

        diagonal_s = min(self.files - s)

        (x, y) = np.zeros(shape=(4)), np.zeros(shape=(4))
        (x_rot, y_rot) = np.zeros(shape=(3,3)), np.zeros(shape=(3,3))

        x = [u[1, int(diagonal_s)], u[2, int(diagonal_s)], u[3, int(diagonal_s)], u[0, int(diagonal_s)]]
        y = [v[1, int(diagonal_s)], v[2, int(diagonal_s)], v[3, int(diagonal_s)], v[0, int(diagonal_s)]]

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

        np.savetxt(self.txt_path + "X2.txt", xRT, fmt='%.8f')
        np.savetxt(self.txt_path + "Y2.txt", yRT, fmt='%.8f')

        print("==> Calibration saved to " + self.txt_path)
        print("==> AXYB has successfully been calculated")

def run():
    config = configparser.ConfigParser()
    config.read("./config.ini")
    # Initialise
    axyb = AXYB(config)
    # Load files
    axyb.load()
    # Calculate dornaika
    axyb.dornaika()

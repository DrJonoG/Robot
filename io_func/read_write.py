import numpy as np
import os

def wMatrix(matrix, filename, path, abbrev=""):
    # Create path if not exists
    if not os.path.exists(path):
        os.makedirs(path)

    # Write matrix file
    mat = np.matrix(matrix)
    file = path + abbrev + filename + '.txt'
    with open(file,'wb') as f:
        for line in mat:
            np.savetxt(f, line, fmt='%.8f')

import numpy as np

def wMatrix(matrix, filename, path, abbrev=""):
    mat = np.matrix(matrix)
    file = path + abbrev + filename + '.txt'
    with open(file,'wb') as f:
        for line in mat:
            np.savetxt(f, line, fmt='%.8f')

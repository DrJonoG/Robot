import numpy as np
from calibration import functions as functions
from PIL import Image, ImageDraw

calibrationPath = r'C:\Users\jonat\OneDrive\Documents\Robot\Example\calibration\axyb\\'
Y = np.loadtxt(calibrationPath + "Y.txt")
X = np.loadtxt(calibrationPath + "X.txt")
intrinsic = np.loadtxt(calibrationPath + "intrinsic.txt")
ttCenter = np.loadtxt(calibrationPath + "center.txt")
rError = np.loadtxt(calibrationPath + '\\rError.txt')
tError = np.loadtxt(calibrationPath + '\\tError.txt')

for i in range(0, 7):


    currentFile = '0000000' + str(i)
    outputFolder = r'C:\Users\jonat\OneDrive\Documents\Robot\Example\model\output\\'
    B = np.loadtxt(r'C:\Users\jonat\OneDrive\Documents\Robot\Example\model\testing\B' + currentFile +'.txt')
    image = r'C:\Users\jonat\OneDrive\Documents\Robot\Example\model\visualize\\' + currentFile + '.jpg'
    degrees = -90



    # Convert to rads
    radians = (degrees * 0.0174532925)
    # get rotation of Y estimate
    estY = np.dot(Y, functions.rotZ(radians))
    # Difference in translation
    ttCenter[2] = 0
    cPoint = ttCenter * -1
    cPoint = np.append(cPoint, [1])
    point = np.dot(functions.rotZ(radians), cPoint)

    #point = (functions.rotZ(radians) * cPoint)
    #point = point[0:4,3]

    # calculate translation back to origin
    point[0:3] = point[0:3] + ttCenter

    # translate to origin
    tempVec =  np.dot(functions.matrix_inverse(Y), point)

    # Create Y
    estY[0:3,3] = tempVec[0:3]

    # Invert back to final Y
    YFinal =  functions.matrix_inverse(estY)


    # Calculate and save A if testing, else not needed
    A = np.dot(X, functions.matrix_inverse(np.dot(YFinal, B)))

    # Adjust for error if exists
    if rError is not None and tError is not None:
        A[0:3,3] = A[0:3,3] + tError
        A[0:3,0:3] = A[0:3,0:3] + rError

    # Estimate projection matrix and save
    AProjection = np.dot(intrinsic, A[0:3,:])
    np.savetxt(outputFolder + currentFile + '.txt', AProjection, fmt='%1.5f')


    #np.savetxt(outputFolder + currentFile + '.txt', A, fmt='%1.5f')
    np.savetxt(outputFolder + "B" + currentFile + '.txt', B, fmt='%1.5f')
    # Output projection (for testing purposes only)
    projectPoint =np.array([0,0,0,1])
    matrix = np.loadtxt(outputFolder + currentFile + '.txt')

    # Project point to camera
    v = (matrix @ projectPoint) # dot product
    x = int(v[0] / v[2])
    y = int(v[1] / v[2])

    # Import an image from directory:
    input_image = Image.open(image)

    # Draw ellipse
    draw = ImageDraw.Draw(input_image)
    draw.ellipse((x-5, y-5, x+5, y+5), fill=(255,0,0,0))

    # Saving the final output
    print("==> saved to " + outputFolder + currentFile + '.jpg')
    input_image.save(outputFolder + currentFile + '.jpg')
    input_image.close()

import numpy as np
import os
from calibration import functions
from PIL import Image, ImageDraw

imageCounter = 0

calibrationFolder = "I:\\calibration2\\axyb\\"
imageFolder = "I:\\calibration2\\testmodel\\visualize\\"
bFolder = "I:\\calibration2\\testmodel\\testing\\"
targetFolder = "I:\\calibration2\\testmodel\\calibtesting\\"
degrees = 0

imagesPerRotation = 32
degreesPerRotation = 90

Y = np.loadtxt(calibrationFolder + "Y.txt")
X = np.loadtxt(calibrationFolder + "X.txt")
intrinsic = np.loadtxt(calibrationFolder + "intrinsic.txt")
ttCenter = np.loadtxt(calibrationFolder + "center.txt")
rError = np.loadtxt(calibrationFolder + '\\rError.txt')
tError = np.loadtxt(calibrationFolder + '\\tError.txt')

for filename in os.listdir(imageFolder):
    # Ignore calibration files
    if ".txt" in filename:
        continue
    f = os.path.join(imageFolder,filename)
    if os.path.isfile(f):
        B = np.loadtxt(bFolder + "B" + filename.replace(".jpg",".txt"))

        if imageCounter % imagesPerRotation == 0 and imageCounter > 0:
            degrees += degreesPerRotation

        # Calculate for a rotation
        if degrees > 0:

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
        else:
            YFinal = Y


        # Calculate and save A if testing, else not needed
        A = np.dot(X, functions.matrix_inverse(np.dot(YFinal, B)))


        # Estimate projection matrix and save
        matrix = np.dot(intrinsic, A[0:3,:])

        # Output projection (for testing purposes only)
        projectPoint =np.array([0,0,0,1])

        # Project point to camera
        v = (matrix @ projectPoint) # dot product

        x = int(v[0] / v[2])
        y = int(v[1] / v[2])

        # Load image as PIL
        input_image = Image.open(imageFolder + "\\" + filename)

        # Draw ellipse
        draw = ImageDraw.Draw(input_image)
        draw.ellipse((x-5, y-5, x+5, y+5), fill=(255,0,0,0))


        # Estimate using error

        # Adjust for error if exists
        if rError is not None and tError is not None:
            A[0:3,3] = A[0:3,3] + tError
            A[0:3,0:3] = A[0:3,0:3] + rError

            # Estimate projection matrix and save
            matrix = np.dot(intrinsic, A[0:3,:])

            # Output projection (for testing purposes only)
            projectPoint =np.array([0,0,0,1])

            # Project point to camera
            v = (matrix @ projectPoint) # dot product
            x = int(v[0] / v[2])
            y = int(v[1] / v[2])

            # Draw ellipse
            draw = ImageDraw.Draw(input_image)
            draw.ellipse((x-5, y-5, x+5, y+5), fill=(0,0,255,0))


        # Saving the final output
        print("==> saved to " + targetFolder + filename + " with rotation of Degrees: " + str(degrees))
        input_image.save(targetFolder + filename)
        input_image.close()

        # Update count
        imageCounter += 1

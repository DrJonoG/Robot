import cv2
import os
import time
import glob
import numpy as np
from PIL import Image, ImageDraw
from threading import Thread

def fetchClass(cameraType):
    if cameraType.lower() == 'logi':
        return Logi()

class Camera(object):
    """
    A class used to represent a camera

    ...

    Attributes
    ----------
    connected : bool
        A boolean indicating whether the camera is connected
    camera : object
        Reference to the camera object relating to the camera in use
    streaming : bool
        A boolean indicating whether the camera is currently streaming a livestream
    recording : bool
        A boolean indicating whether the camera is currently recording

    Methods
    -------
    connect(id)
        Prints the animals name and what sound it makes
    takephoto()
        Captures a photo
    recordvideo()
        Records a video
    savemedia(media, file_path, type)
        Saves media in forms of video or image
    livestream()
        Initialise the live stream and return the thread
    fetchframe()
        Update the livestream by continuously fetching frames
    disconnect()
        Disconnect opterations relating to the camera
    stop()
        Stops the livestream
    """

    def __init__(self):
        self.connected = False
        self.camera = None
        self.streaming = False
        self.recording = False
        self.path = None

    def connect(self, id="0"):
        raise NotImplementedError("camera.connect has not been implemented")

    def capture(self):
        raise NotImplementedError("camera.takephoto has not been implemented")

    def recordvideo(self):
        raise NotImplementedError("camera.recordvideo has not been implemented")

    def savemedia(self, media, file_path,type="image"):
        if not os.path.exists(file_path):
            os.makedirs(file_path)
        if type == "image":
            # Count number of jpg images to name files correctly.
            numFiles = len(glob.glob1(file_path,"*.jpg"))
            file_name = str(numFiles).zfill(8)
            cv2.imwrite(file_path + file_name + ".jpg", media)
            print("==> Image saved: %s" % file_name + ".jpg")
            return file_name

    def livestream(self):
        if not self.connected:
            print("==> Error: Not connected to a camera.")
            return None
        elif self.streaming:
            print("==> Error: Live stream already on. Returning.")
            return None

        # Streaming beginning, set to true
        self.streaming = True
        # If connected and not already streaming, begin live stream
        print("==> Displaying live feed")

        # Return thread to update livestream
        return Thread(target=self.fetchframe, args=(), daemon=True)

    def projectPoint(self, point, camera, image, output):
        matrix = np.loadtxt(camera)

        # Project point to camera
        v = (matrix @ point) # dot product
        x = int(v[0] / v[2])
        y = int(v[1] / v[2])

        # Import an image from directory:
        input_image = Image.open(image)

        # Draw ellipse
        draw = ImageDraw.Draw(input_image)
        draw.ellipse((x-5, y-5, x+5, y+5), fill=(255,0,0,0))

        # Saving the final output
        input_image.save(output)

    def fetchframe(self):
        raise NotImplementedError("camera.fetchframe has not been implemented")

    def disconnect(self):
        raise NotImplementedError("camera.disconnect has not been implemented")

    def stop_stream(self):
        self.streaming = False

    def stop_recording(self):
        self.recording = False

    def msg_connected(self):
        print("==> Camera connected")
        self.connected = True

    def msg_disconnected(self):
        print("==> Camera disconnected")
        self.connected = False

class Canon(Camera):
    pass


# Camera function for traditional webcams
class Logi(Camera):
    def connect(self,path="./captures/", camera_id=0):
        try:
            self.camera = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        except:
            print("==> Error: Cannot open webcam (ID: " + str(camera_id) + "), Please specify integer value")
            return False
        # Check camera connected
        if not self.camera.isOpened():
            print("==> Error: Cannot open webcam (ID: " + str(camera_id) + ")")
            return False
        else:
            self.path = path
            self.msg_connected()

    def fetchframe(self):
        (_, frame) = self.camera.read()
        # Open live stream
        while self.streaming:
            cv2.imshow('Input', frame)
            (_, frame) = self.camera.read()
            cv2.waitKey(1)
        self.disconnect()
        # Once finished, destroy windows
        cv2.destroyAllWindows()
        print("==> Camera auccesfully terminated live stream")

    def recordvideo(self, savepath=None):
        frames = []
        self.recording = True
        while self.recording:
            (grabbed, frame) = self.camera.read()
            if grabbed:
                frames.append(frame)
        return

    # Returns the name of the file it saved
    def capture(self):
        (grabbed, frame) = self.camera.read()
        if grabbed:
            return self.savemedia(frame, self.path)

    def disconnect(self):
        self.camera.release()
        self.msg_disconnected()

import cv2
import os
import time
import glob
import numpy as np
from PIL import Image, ImageDraw
from threading import Thread
from datetime import datetime

def fetchClass(cameraType, config):
    if cameraType.lower() == 'logi':
        return Logi(config)

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

    def __init__(self, config=None):
        self.camera_id = 0
        self.connected = False
        self.camera = None
        self.streaming = False
        self.recording = False
        self.config = config
        self.path = self.config['camera']['save_path']

    def connect(self, id="0"):
        raise NotImplementedError("camera.connect has not been implemented")

    def capture(self):
        raise NotImplementedError("camera.takephoto has not been implemented")

    def recordvideo(self):
        raise NotImplementedError("camera.recordvideo has not been implemented")

    def savemedia(self, media, type="image"):
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        if type == "image":
            # Count number of jpg images to name files correctly.
            numFiles = len(glob.glob1(self.path,"*.jpg"))
            file_name = str(numFiles).zfill(8)
            # Rotate view
            #media = cv2.rotate(media, cv2.ROTATE_90_CLOCKWISE)
            cv2.imwrite(self.path + file_name + ".jpg", media,  [cv2.IMWRITE_JPEG_QUALITY, 100])
            #print(f"==> Image saved: {self.path + file_name}.jpg")
            return file_name

    def livestream(self):
        if not self.connected:
            print(datetime.now().strftime('%H:%M:%S') + " ==> Error: Not connected to a camera.")
            return None
        elif self.streaming:
            print(datetime.now().strftime('%H:%M:%S') + " ==> Error: Live stream already on. Returning.")
            return None

        # Streaming beginning, set to true
        self.streaming = True
        # If connected and not already streaming, begin live stream
        print(datetime.now().strftime('%H:%M:%S') + " ==> Displaying live feed")

        # Return thread to update livestream
        Thread(target=self.fetchframe, args=(), daemon=True).start()

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
        width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(datetime.now().strftime('%H:%M:%S') + " ==> Camera connected. Resolution (" + str(width) + " x " + str(height) + ")" )
        self.connected = True

    def msg_disconnected(self):
        print(datetime.now().strftime('%H:%M:%S') + " ==> Camera disconnected")
        self.connected = False

    def rescale(self, frame, scale):
        width = int(frame.shape[1] * scale)
        height = int(frame.shape[0] * scale)
        return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


class Canon(Camera):
    pass


# Camera function for traditional webcams
class Logi(Camera):
    def connect(self,camera_id=0):
        self.camera_id = camera_id
        if(self.connected):
            return True
        else:
            try:
                self.camera = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 3840.0)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160.0)
                time.sleep(1)
            except:
                print(datetime.now().strftime('%H:%M:%S') + " ==> Error: Cannot open webcam (ID: " + str(camera_id) + "), Please specify integer value")
                return False
            # Check camera connected
            if not self.camera.isOpened():
                print(datetime.now().strftime('%H:%M:%S') + " ==> Error: Cannot open webcam (ID: " + str(camera_id) + ")")
                return False
            else:
                self.msg_connected()

    def fetchframe(self):
        (_, frame) = self.camera.read()
        # Open live stream
        while self.streaming:
            try:
                scale_percent = 20 # percent of original size
                width = int(frame.shape[1] * scale_percent / 100)
                height = int(frame.shape[0] * scale_percent / 100)
                dim = (width, height)
                # resize image
                frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
                # Rotate view
                #frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

                cv2.imshow('Input', frame)
                (_, frame) = self.camera.read()
                cv2.waitKey(1)
            except Exception:
                continue
        self.disconnect()
        # Once finished, destroy windows
        cv2.destroyAllWindows()
        print(datetime.now().strftime('%H:%M:%S') + " ==> Camera auccesfully terminated live stream")

    def recordvideo(self, savepath=None):
        frames = []
        self.recording = True
        while self.recording:
            (grabbed, frame) = self.camera.read()
            if grabbed:
                frames.append(frame)
        return

    # Returns the name of the file it saved
    def capture(self, bw=False):
        attempts = 0
        # Initial test grab to ensure params are right
        (grabbed, frame) = self.camera.read()
        while True:
            #blur = False
            (grabbed, frame) = self.camera.read()
            # Check frame for blur
            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #if cv2.Laplacian(gray, cv2.CV_64F).var() < 1:
            #    blur = True
            #    attempts = attempts + 1
            #    print(datetime.now().strftime('%H:%M:%S') + f" ==> Image blurred. Attempting recapture {attempts} of 10.")
            #    time.sleep(1)
            ## Exit when non blur is found
            #if (grabbed and blur == False) or attempts >= 10:
                #attempts = 0
                #if bw:
                #    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                #return self.savemedia(frame)
            if bw:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return self.savemedia(frame)

    def disconnect(self):
        self.camera.release()
        self.msg_disconnected()

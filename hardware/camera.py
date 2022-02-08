import cv2
import os
import time
from threading import Thread

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

    def connect(self, id):
        raise NotImplementedError("camera.connect has not been implemented")

    def takephoto(self):
        raise NotImplementedError("camera.takephoto has not been implemented")

    def recordvideo(self):
        raise NotImplementedError("camera.recordvideo has not been implemented")

    def savemedia(self, media, file_path,type="image"):
        if not os.path.exists(file_path):
            os.makedirs(file_path)
        if type == "image":
            file_name = file_path + str(int(time.time()*1000.0)) + ".jpg"
            cv2.imwrite(file_name, media)
            print("==> Image saved: %s" % file_name)

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


    def fetchframe(self):
        raise NotImplementedError("camera.fetchframe has not been implemented")

    def disconnect(self):
        raise NotImplementedError("camera.disconnect has not been implemented")

    def stop_stream(self):
        self.streaming = False

    def stop_recording(self):
        self.recording = False

class Canon(Camera):
    pass

# Camera function for traditional webcams
class Logi(Camera):
    def connect(self, camera_id):
        if isinstance(camera_id[0], int):
            camera_id = int(camera_id[0])
            self.camera = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
        else:
            print("==> Error: Cannot open webcam (ID: " + str(camera_id) + "), Please specify integer value")
            return False
        # Check camera connected
        if not self.camera.isOpened():
            print("==> Error: Cannot open webcam (ID: " + str(camera_id) + ")")
            return False
        else:
            print("==> Succesfully connected")
            self.connected = True

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
        print("==> Succesfully terminated live stream")

    def recordvideo(self, savepath=None):
        frames = []
        self.recording = True
        while self.recording:
            (grabbed, frame) = self.camera.read()
            if grabbed:
                frames.append(frame)
        return

    def takephoto(self, savepath=None):
        (grabbed, frame) = self.camera.read()
        if grabbed:
            if savepath == None: savepath = "./captures/"
            self.savemedia(frame, savepath)

    def disconnect(self):
        self.camera.release()

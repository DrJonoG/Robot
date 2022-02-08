class Robot(object):
    def __init__(self):
        self.connected = False
        self.moving = False
        self.position = None

    def connect(self, address):
        raise NotImplementedError("robot.connect has not been implemented")

    def move(self, position):
        raise NotImplementedError("robot.move has not been implemented")

    def disconnect(self):
        raise NotImplementedError("robot.disconnect has not been implemented")

class UR5(Robot):
    pass

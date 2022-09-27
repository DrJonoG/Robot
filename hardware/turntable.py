import time
from win32com.client import Dispatch
import pythoncom

def fetchClass(turntableType):
    if turntableType.lower() == 'hrt':
        return HRT()


class Turntable(object):
    def __init__(self):
        pass

    def connect(self):
        pass

    def GoToPos(self):
        pass

    def GotoCW(self):
        pass

    def GotoCCW(self):
        pass

    def disconnect(self):
        pass

class HRT(Turntable):
    def __init__(self):
        self.turntable = None
        self.connected = False

    def connect(self):
        # Check if we are already connected to the turntable
        if self.connected:
            return

        # Needed as turntable is running within thread
        pythoncom.CoInitialize()
            ## Dispatching TurnTableControl
        ttc = Dispatch("TurnTableControlLib.TurnTableControl")

        ## Check if there is a cable / TurnTable connected
        if ttc.Count == 0:
            raise Exception('No TurnTable is connected')

        self.turntable = ttc.TurnTables(0) ## select first turntable or better said cable

        if self.turntable.ConnectionState == 0:
            raise Exception('==> Error: No CUD III cable is connected')
        elif self.turntable.ConnectionState == 1:
            raise Exception('==> Error: The Turntable is switched off or not connected to CUD III')
        elif self.turntable.ConnectionState == 2:
            self.connected = True
            print('==> Successfully connected to turntable')

        # initialise velocity
        self.turntable.Velocity = 0.2
        self.turntable.Torque = 20
        # Reset
        self.Home()

    def waitWhileDriving(self):
        time.sleep(1)
        secondsWaited = 0.0
        while self.turntable.IsMoving:
            time.sleep(0.5)
            secondsWaited = secondsWaited + 0.5
            if secondsWaited > 120:
                print('==> Error: Timeout while wait for turntable to stop')
                break

    def Home(self):
        self.turntable.GotoPosition(0)
        self.waitWhileDriving()
        time.sleep(0.5)

    def GoTo(self, x):
        self.turntable.GotoPosition(x)
        self.waitWhileDriving()
        time.sleep(0.5)

    def GotoCW(self, x):
        self.turntable.GotoCW(x)
        self.waitWhileDriving()
        time.sleep(0.5)

    def GotoCCW(self, x):
        self.turntable.GotoCCW(x)
        waitWhileDriving(self.turntable)
        time.sleep(5)

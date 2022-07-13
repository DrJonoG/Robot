import time
from win32com.client import Dispatch

def waitWhileDriving(TT):
    time.sleep(1)
    secondsWaited = 0.0
    while TT.IsMoving:
        time.sleep(0.5)
        secondsWaited = secondsWaited + 0.5
        if secondsWaited > 120:
            print('Timeout while wait for turntable to stop')
            break

## Dispatching TurnTableControl
ttc = Dispatch("TurnTableControlLib.TurnTableControl")

## Check if there is a cable / TurnTable connected
if ttc.Count == 0:
    raise Exception('No TurnTable is connected')

TT = ttc.TurnTables(0) ## select first turntable or better said cable

if TT.ConnectionState == 0:
    raise Exception('No CUD III cable is connected')
elif TT.ConnectionState == 1:
    raise Exception('The Turntable is switched off or not connected to CUD III')
elif TT.ConnectionState == 2:
    print('The Turntable has been found!')

## ------------------------- Example of commands ------------------------
    
## ----- TurnTable Info -----
# read the name of turntable
name = TT.Name
print(name)
    
# read Firmware Version
firmware = TT.FirmwareVersion
print('Firmware version: ', firmware)
    
# read TurnTable Type
ttType = TT.TurnTableType
    
## ----- Moving to Position -----
# go to position X
x = 0
print('Going to position', x)
TT.GotoPosition(x)
waitWhileDriving(TT)
time.sleep(5)
    
# go to position X clock wise
x = -60
print('Going to position {} clockwise'.format(x))
TT.GotoCW(x)
waitWhileDriving(TT)
time.sleep(5)
    
# go to position X counter clock wise
x = 60
print('Going to position {} counter clockwise'.format(x))
TT.GotoCCW(x)
waitWhileDriving(TT)
time.sleep(5)
    
# read current position
current_position = TT.Position
print('Current Position: ', current_position)
    
# check if Turntable is moving
isMoving = TT.IsMoving
print('Turntable is moving: ', isMoving)
    
# stop Turntable while moving
print('Goint to position 0')
TT.GotoPosition(0)  # go to position X clock wise
time.sleep(2)
TT.MoveAbort()  # stop
print('STOP')
    
time.sleep(5)
    
# going by steps
TT.StepSize = 5
TT.StepCW() # step clock wise
time.sleep(3)
TT.StepCCW() #step counter clock wise
    
# save current position as 0 position
print('Saving current position as 0 position')
TT.SetOrigin()
    
## ----- Polarity -----
## display Polarity (bipolar -180 - +180, unipolar 0 - 360)
polarity = TT.DisplayPolarity
print('Polarity: ', polarity)
    
# change display Polarity
TT.DisplayPolarity = 0  # Unipolar
TT.GotoPosition(-90)
polarity = TT.DisplayPolarity
print('Polarity: ', polarity)
waitWhileDriving(TT)
time.sleep(3)
    
TT.DisplayPolarity = 1  # Bipolar
TT.GotoPosition(0)
polarity = TT.DisplayPolarity
print('Polarity: ', polarity)
waitWhileDriving(TT)
    
## ----- Velocity -----
    
# get velocity
velocity = TT.Velocity
    
# set velocity 0.1 - 2.0
TT.Velocity = 1.5
TT.GotoPosition(-50)
waitWhileDriving(TT)
time.sleep(3)
    
TT.Velocity = 1
TT.GotoPosition(0)
waitWhileDriving(TT)
time.sleep(3)
    
TT.Velocity = 0.7
TT.GotoPosition(50)
waitWhileDriving(TT)
time.sleep(3)
    
TT.Velocity = 0.3
TT.GotoPosition(0)
waitWhileDriving(TT)
time.sleep(3)
    
TT.Velocity = 0.5
TT.GotoPosition(-40)
waitWhileDriving(TT)
    
TT.SetOrigin() # save current position as 0 position
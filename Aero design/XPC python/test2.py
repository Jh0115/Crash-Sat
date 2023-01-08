import sys
import xpc
import time
from simple_pid import PID

client = xpc.XPlaneConnect()

#set up the drop altitude, orientation, speed, and wing sweep
client.sendPOSI([-998,-998,4267,0,180,0])
client.sendDREFs(["sim/flightmodel2/controls/wingsweep_ratio","sim/cockpit2/controls/wingsweep_ratio"],[1,1])
velRefs = ["sim/flightmodel/position/local_vx","sim/flightmodel/position/local_vy","sim/flightmodel/position/local_vz",]
client.sendDREFs(velRefs,[0,0,-100])

#while above 200 meters just dive without control surface input
alt = 2000
while (alt>350):
    pose = client.getPOSI()
    alt = pose[2]

    client.sendCTRL([0,0,0])
    time.sleep(0.05)

#pitch with a constant 6 Gs until pitch is 0 degrees (currently no controller just half pitch
pitch = -90
client.sendDREFs(["sim/flightmodel2/controls/wingsweep_ratio","sim/cockpit2/controls/wingsweep_ratio"],[0,0])

while (pitch<0):
    pose = client.getPOSI()
    pitch = pose[3]

    client.sendCTRL([0.7,0,0])
    time.sleep(0.05)

#hover at constant altitude
print("hover mode begun")
pose = client.getPOSI()
altSP = pose[2]

pidPitch = PID(0.2,0,0.05,setpoint=0)
pidRoll = PID(0.001,0,0,setpoint=0)

while True:
    pose = client.getPOSI()
    alt = pose[2]
    roll = pose[4]

    elev = pidPitch((alt-altSP))
    aeil = pidRoll(roll)

    if elev>1:
        elev = 1
    if elev<-1:
        elev = -1

    if aeil>1:
        aeil = 1
    if aeil<-1:
        aeil = -1
    
    client.sendCTRL([elev,aeil,0])
    print(str(altSP) + "," + str(alt))
    

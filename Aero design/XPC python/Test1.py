import sys
import math
import time
import xpc
from simple_pid import PID

pidRoll = PID(0.005,0,0,setpoint=0)
pidPitch = PID(0.005,0.0001,0,setpoint=-85)

print("Altitude (m), Airspeed(m/s)")

with xpc.XPlaneConnect() as client:
    while True:
        ## grab orientation
        pose = client.getPOSI()

        vz = client.getDREF("sim/flightmodel/forces/vz_air_on_acf")
        vy = client.getDREF("sim/flightmodel/forces/vy_air_on_acf")
        vx = client.getDREF("sim/flightmodel/forces/vx_air_on_acf")

        spd = math.sqrt(vx[0]**2+vy[0]**2+vz[0]**2)
        
        roll = pose[4]
        elev = pose[3]

        ## put into pid controllers
        aeil = pidRoll(roll)
        elev = pidPitch(elev)

        ## send controls to x plane
        if aeil>1:
            aeil = 1
        if aeil<-1:
            aeil = -1

        if elev>1:
            elev = 1
        if elev<-1:
            elev = -1
            
        ctrl = [elev,aeil,0,0]
        client.sendCTRL(ctrl)

        print(str(pose[2]) + "," + str(spd))

        ## sleep for a sec
        time.sleep(0.05)

          


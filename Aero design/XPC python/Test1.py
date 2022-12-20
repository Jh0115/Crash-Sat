import sys
import math
import time
import xpc
from simple_pid import PID

rollPNT = 25

pidRoll = PID(0.01,0,0,setpoint=rollPNT)
pidPitch = PID(0.05,0.02,0.001,setpoint=5)
pidThrottle = PID(0.08,0.02,0.5,setpoint=180)

t1 = time.time()

with xpc.XPlaneConnect() as client:
    while True:
        ## grab orientation
        pose = client.getPOSI()

        vz = client.getDREF("sim/flightmodel/forces/vz_air_on_acf")
        vy = client.getDREF("sim/flightmodel/forces/vy_air_on_acf")
        vx = client.getDREF("sim/flightmodel/forces/vx_air_on_acf")

        spd = math.sqrt(vx[0]**2+vy[0]**2+vz[0]**2)*1.94384

        print('Roll:  ' + str(pose[4]))
        print('Pitch: ' + str(pose[3]))
        print('Speed:  ' + str(spd) + '\n')
        
        roll = pose[4]
        elev = pose[3]

        ## put into pid controllers
        aeil = pidRoll(roll)
        elev = pidPitch(elev)
        thrt = pidThrottle(spd)

        ## send controls to x plane
        if aeil>1:
            aeil = 1
        if aeil<-1:
            aeil = -1

        if elev>1:
            elev = 1
        if elev<-1:
            elev = -1

        if thrt>1:
            thrt = 1
        if thrt<0.001:
            thrt = 0.001

        print('Throttle: ' + str(thrt) + '\n')
        ctrl = [elev,aeil,0,thrt]
        client.sendCTRL(ctrl)

        ## sleep for a sec
        time.sleep(0.05)

        t2 = time.time()
        if (t2-t1)>60:
            print('Controller reset!\n')
            t1 = t2
            rollPNT = -rollPNT
            pidRoll = PID(0.01,0,0,setpoint=rollPNT)
            pidPitch = PID(0.05,0.02,0.001,setpoint=5)
            pidThrottle = PID(0.08,0.02,0.5,setpoint=180)

          


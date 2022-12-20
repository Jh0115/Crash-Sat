## Xplane waypoint travel controller

import sys
import math
import time
import xpc
import numpy as np
from simple_pid import PID

def get_bearing(lat1, long1, lat2, long2):
    dLon = (long2 - long1)
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
    brng = np.arctan2(x,y)
    brng = np.degrees(brng)

    return brng

def takeoff():
    gear_flag = True
    TO_pitch = 20
    pose = client.getPOSI()
    head_tar = pose[5]
    
    pidPitch = PID(0.05,0,0,setpoint=TO_pitch)
    pidRoll = PID(0.01,0,0,setpoint=0)
    pidSteer = PID(0.2,0,0,setpoint=0)
    
    alt0 = pose[2]
    while True:
        #attempt to pitch the nose 10 degrees until altitude has increased by 100 meters
        pose = client.getPOSI()
        alt = pose[2]
        pitch = pose[3]
        roll = pose[4]
        yaw = pose[5]

        shift = 360-head_tar
        head_shift = yaw+shift
        if head_shift>=360:
            head_shift = head_shift-360
        
        if head_shift>180:
            d_heading = head_shift-360
        else:
            d_heading = head_shift
    
        aeil = pidRoll(roll) #decide aeileron and elevator needs
        elev = pidPitch(pitch)
        rudd = pidSteer(d_heading)

        #protect against stupids
        if aeil>1:
            aeil = 1
        if aeil<-1:
            aeil = -1

        if elev>1:
            elev = 1
        if elev<-1:
            elev = -1

        if rudd>1:
            rudd = 1
        if rudd<-1:
            rudd = -1
    
        ctrl = [elev,aeil,rudd,0.95] #update the needs in the sim
        client.sendCTRL(ctrl)

        if ((alt-alt0)>20 and gear_flag):
            #raise the landing gear
            client.sendCTRL([-998,-998,-998,-998,0,-998,-998])
            gear_flag = False

        if (alt-alt0)>300:
            break
        
        time.sleep(0.1)

def changeHeading(head_tar, head_tol):
    ## Change the heading to a desired tolerance

    pidHead = PID(5,0,0,setpoint=0) #this PID changes the setpoint for the roll controller
    pidPitch = PID(0.05,0.02,0.001,setpoint=0)
    rollPNT = 0

    iterCounter = 0
    
    while True:
        #print('Roll setpoint: ' + str(rollPNT))
        pidRoll = PID(0.01,0,0,setpoint=rollPNT) #update the roll controller setpoint
        
        pose = client.getPOSI() #get heading information
        elev = pose[3]
        roll = pose[4]
        heading = pose[5]
        #print('Heading:       ' + str(heading))

        shift = 360-head_tar
        head_shift = heading+shift
        if head_shift>=360:
            head_shift = head_shift-360
        
        if head_shift>180:
            d_heading = head_shift-360
        else:
            d_heading = head_shift

        #print('Delta-yaw:     ' + str(d_heading))

        aeil = pidRoll(roll) #decide aeileron and elevator needs
        elev = pidPitch(elev)

        #protect against stupids
        if aeil>1:
            aeil = 1
        if aeil<-1:
            aeil = -1

        if elev>1:
            elev = 1
        if elev<-1:
            elev = -1

        ctrl = [elev,aeil,0,0.95] #update the needs in the sim
        client.sendCTRL(ctrl)

        #if the heading is within the desired tolerance for a few iterations exit the function
        
        if (abs(d_heading)<head_tol):
            #print('Almost there...')
            iterCounter = iterCounter+1
            if iterCounter>=10:
                break
        else:
            #print('Almost there...')
            iterCounter = 0

        #if we are not there yet, update the rollPNT variable under the PID control and keep going
        rollPNT = pidHead(d_heading)

        #never more than a 2G turn
        if rollPNT>60:
            rollPNT = 60
        if rollPNT<-60:
            rollPNT = -60

        #print('')
        #print('')
        time.sleep(0.1) #short delay

def fly2waypoint(lat_tar,long_tar,alt_tar,speed,lat_tol,long_tol,alt_tol):
    
    ## Using the given gps coordinate and speed, fly to the desired waypoint in a straight line
    pidRoll = PID(0.01,0,0,setpoint=0) #roll control
    pidHead = PID(0.02,0,0,setpoint=0) #yaw control
    pidAlt = PID(0.05,0.01,0,setpoint=0)
    pidSpeed = PID(0.08,0.02,0.5,setpoint=speed)
    pitchPNT = 0 #the target pitch angle
    
    while True:
        #get state of craft
        pose = client.getPOSI()
        lat = pose[0]
        long = pose[1]
        head_tar = get_bearing(lat,long,lat_tar,long_tar)

        alt = pose[2]

        pitch = pose[3]
        roll = pose[4]
        heading = pose[5]
        
        shift = 360-head_tar
        head_shift = heading+shift
        if head_shift>=360:
            head_shift = head_shift-360
        
        if head_shift>180:
            d_heading = head_shift-360
        else:
            d_heading = head_shift

        vz = client.getDREF("sim/flightmodel/forces/vz_air_on_acf")
        vy = client.getDREF("sim/flightmodel/forces/vy_air_on_acf")
        vx = client.getDREF("sim/flightmodel/forces/vx_air_on_acf")

        spd = math.sqrt(vx[0]**2+vy[0]**2+vz[0]**2)*1.94384
        
        #get updated control surfaces from state pid controllers
        pidPitch = PID(0.08,0,0,setpoint=pitchPNT) #===============================
        elev = pidPitch(pitch)
        aeil = pidRoll(roll)
        rudd = pidHead(d_heading)
        thrt = pidSpeed(spd)

        if aeil>1:
            aeil = 1
        if aeil<-1:
            aeil = -1

        if elev>1:
            elev = 1
        if elev<-1:
            elev = -1

        if rudd>1:
            rudd = 1
        if rudd<-1:
            rudd = -1

        if thrt>1:
            thrt = 1
        if thrt<0.001:
            thrt = 0.001

        #update the control surfaces
        ctrl = [elev,aeil,rudd,thrt]
        client.sendCTRL(ctrl)

        #given altitude, update pitch setpoint from altitude PID control
        d_alt = alt_tar-alt
        pitchPNT = pidAlt(-d_alt)

        if pitchPNT>45:
            pitchPNT = 45
        if pitchPNT<-30:
            pitchPNT = -30

        d_lat = abs(lat-lat_tar)
        d_long = abs(long-long_tar)
        if (d_lat<lat_tol) and (d_long<long_tol) and (abs(d_alt)<alt_tol):
            break
        
        time.sleep(0.01)

#-------------------------------------------------------------------------------
## Start the script

# Step 1: define waypoints
pt1 = [47.482092,-122.410818,400]
pt2 = [47.45331327325336, -122.3098809736118,250]
#pt3 =
#pt4 = 

# Step 2: takeoff

with xpc.XPlaneConnect() as client:
    takeoff()

    pose = client.getPOSI()
    beta = get_bearing(pose[0],pose[1],pt1[0],pt1[1])
    print('New heading')

    changeHeading(beta,0.5)

    print('Heading attained')
    fly2waypoint(pt1[0],pt1[1],pt1[2],200,0.001,0.001,50)
#================================================================
    pose = client.getPOSI()
    beta = get_bearing(pose[0],pose[1],pt2[0],pt2[1])
    print('New heading')

    changeHeading(beta,0.5)

    print('Heading attained')
    fly2waypoint(pt2[0],pt2[1],pt2[2],170,0.001,0.001,50)




    

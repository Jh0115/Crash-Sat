import sys
import xpc
import random as rd
import pandas as pd
import math
import time

print("Control experiment")
##============================================================
client = xpc.XPlaneConnect()
        
windRefs = ["sim/weather/wind_altitude_msl_m[0]",
            "sim/weather/wind_altitude_msl_m[1]",
            "sim/weather/wind_altitude_msl_m[2]",
            "sim/weather/wind_direction_degt[0]",
            "sim/weather/wind_direction_degt[1]",
            "sim/weather/wind_direction_degt[2]",
            "sim/weather/wind_speed_kt[0]",
            "sim/weather/wind_speed_kt[1]",
            "sim/weather/wind_speed_kt[2]",
            "sim/weather/turbulence[0]",
            "sim/weather/turbulence[1]",
            "sim/weather/turbulence[2]"]

velRefs = ["sim/flightmodel/forces/vx_air_on_acf","sim/flightmodel/forces/vy_air_on_acf","sim/flightmodel/forces/vz_air_on_acf"]

for x in range(15):
    filename = 'dataControl' + str(x) + '.csv'
    df = pd.DataFrame(columns=['Altitude (m)','Airspeed (m/s)'])

    wind1 = rd.randrange(0,1200)
    wind2 = rd.randrange(1300,2400)
    wind3 = rd.randrange(2500,3600)
    wind4 = rd.uniform(0.0,359.9)
    wind5 = rd.uniform(0.0,359.9)
    wind6 = rd.uniform(0.0,359.9)
    wind7 = 0#rd.randrange(0,20)
    wind8 = 0#rd.randrange(0,20)
    wind9 = 0#rd.randrange(0,20)
    wind10 = 0#rd.randrange(0,10)
    wind11 = 0#rd.randrange(0,10)
    wind12 = 0#rd.randrange(0,10)

    windVals = [wind1,
                wind2,
                wind3,
                wind4,
                wind5,
                wind6,
                wind7,
                wind8,
                wind9,
                wind10,
                wind11,
                wind12]

    #set the winds and altitude to the desired value
    client.sendDREFs(windRefs,windVals)
    client.sendPOSI([-998,-998,4267,0,0,0])

    #while altitude is above 200 meters append data to the primary dataframe
    alt = 9999
    #unpause the simulation
    client.pauseSim(False)

    print(filename)
    
    while alt>200:
        #grab altitude and airspeed from dataRefs
        v_vec = client.getDREFs(velRefs)
        spd = math.sqrt(v_vec[0][0]**2+v_vec[1][0]**2+v_vec[2][0]**2)

        pose = client.getPOSI()
        alt = pose[2]

        df.loc[len(df.index)] = [alt,spd]

        time.sleep(0.05)

    #pause the simulation, save the csv
    client.pauseSim(True)
    df.to_csv(filename,index=False)

    drag = client.getDREF("sim/flightmodel/forces/faxil_aero")
    print(str(alt),str(spd),str(drag))


























import serial as ps
import csv
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#===================================================================================================
#step 1: Define functions
def any_comp(l,s): #do any values in list l match the value s
    comp = False
    for x in l:
        if x==s:
            comp = True

    return comp
        
def validateSerialData(dataList):
    #this function will analyze data from the serial port to check
    #if it is actual measurements or garbage measurements
    approval = True
    listLength = len(dataList)
    nullVal = 99999

    if not (isinstance(dataList,list)):
        approval = False
        
    #if len(dataList!=5):
    #    approval = False
    
    if any_comp(dataList,nullVal) and approval: #if any values are null
        approval = False

    if any_comp(dataList,8388608) or any_comp(dataList,-8388607) and approval: #if load cell values are saturated
        aproval = False
        
    return approval

##def changeSerialPort(portObject,portName): #probably will change this to a drop-down list later
##    portObject.close()
##    time.sleep(0.25)
##    arduinoPort = ps.Serial(portName,57600)
##    arduinoPort.flushInput()
##    arduinoPort.flushOutput()
##    time.sleep(0.25)
##
##    return arduinoPort

#===================================================================================================
#step 2: Initialize Heads Up Display
fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots()

#step 3: Initialize CSV files
epoch = str(time.time()) #epoch time appended to file names for uniquness

with open(''.join(['data-full-',epoch,'.csv']),'w',newline='') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames=["Time","Dynamic Press","Load 1","Load 2","Load 3"])
    csv_writer.writeheader()

with open(''.join(['data-steady-',epoch,'.csv']),'w',newline='') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames=["Time","Airspeed","Lift Coeff","Drag Coeff","Moment Coeff"])
    csv_writer.writeheader()

#step 4: Initialize Serial port connection
global arduinoPort
arduinoPort = ps.Serial('COM4',57600) #'COM4' is default value but can be changed by drop down list later
arduinoPort.flushInput()
arduinoPort.flushOutput()
time.sleep(0.25) #give computer a moment to flush the port

#==================================================================================================
#step 5: Main loop
   
while True:
    #testing area
   
##    #Step 2: get data from serial port register
##    arduinoPort.flushInput()
##    arduinoPort.flushOutput()
##
##    time.sleep(0.01)
##
##    data = arduinoPort.readline()
##    data_str = str(data.decode("utf-8"))
##    vals = list(map(int, data_str.split(',')))
##
##    if validateSerialData(vals):
##        #if the data is valid save it
##        print(vals)
##
##        #write data to full CSV
##        with open(''.join(['data-full-',epoch,'.csv']),'a',newline='') as csv_file_full:
##            csv_full = csv.DictWriter(csv_file_full,fieldnames=["Time","Dynamic Press","Load 1","Load 2","Load 3"])
##
##            info = {"Time":vals[0],"Dynamic Press":vals[1],"Load 1":vals[2],"Load 2":vals[3],"Load 3":vals[4]}
##            csv_full.writerow(info)
##
##        #write data to steady state CSV
##        with open(''.join(['data-steady-',epoch,'.csv']),'a',newline='') as csv_file_steady:
##            csv_steady = csv.DictWriter(csv_file_steady,fieldnames=["Time","Airspeed","Lift Coeff","Drag Coeff","Moment Coeff"])

        














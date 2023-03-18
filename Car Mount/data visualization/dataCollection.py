#data collection file for watching the serial port and saving data to a CSV file
import serial as ps
import csv
import time

#step 1: initialize
arduinoPort = ps.Serial('COM4',57600)
arduinoPort.flushInput()
arduinoPort.flushOutput()

time.sleep(3)
fieldnames = ["Time","Readout"]

with open('data.csv','w',newline='') as csv_file:
    csv_writer = csv.DictWriter(csv_file,fieldnames=fieldnames)
    csv_writer.writeheader()

print("Collecting data...")

while True:
    with open('data.csv','a',newline='') as csv_file:
        csv_writer = csv.DictWriter(csv_file,fieldnames=fieldnames)

        #Step 2: get data from serial port register
        arduinoPort.flushInput()
        arduinoPort.flushOutput()

        time.sleep(0.01)

        data = arduinoPort.readline()

        #clear the serial buffer
        
        #convert to usable data
        data_str = str(data.decode("utf-8"))
        vals = list(map(int, data_str.split(',')))

        if len(vals)==2:

            print(vals)

            #Step 3: append data
            info = {"Time":vals[0],"Readout":vals[1]}
            csv_writer.writerow(info)

        #Step 4: delay loop


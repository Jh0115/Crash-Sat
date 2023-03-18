import serial as ps
import matplotlib.pyplot as plt
import time
from matplotlib.animation import FuncAnimation

x_vals = []
y_vals = []
index = count()

def animate(val):
    x_vals.append(next(index))
    y_vals.append(val)

    plt.plot(x_vals,y_vals)

arduinoPort = ps.Serial('COM4',57600)

time.sleep(5)
num = []

ani = FuncAnimation(plt.gcf(),animate,interval=1000)
plt.show()

while True:
    #get data from serial port register

    data = arduinoPort.readline()
    integerForm = int(data.decode("utf-8"))

    print(integerForm)

    #append data to end of lists

    num.append(integerForm)

    #add new data point to living plot

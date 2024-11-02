import serial
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

ser = serial.Serial('COM3', 115200)
plt.style.use('fivethirtyeight')

#lists for sensor data
time_data = []
mq135_data = []
ammonia_data = []
co2_data = []

def update(frame):
    if ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        values = line.split(',')
        
        try:
            #append new data
            mq135_value = int(values[0])
            ammonia_value = float(values[1])
            co2_value = float(values[2])
            
            time_data.append(frame)
            mq135_data.append(mq135_value)
            ammonia_data.append(ammonia_value)
            co2_data.append(co2_value)

            # Limit data length to the last 100 readings
            if len(time_data) > 100:
                time_data.pop(0)
                mq135_data.pop(0)
                ammonia_data.pop(0)
                co2_data.pop(0)

            #clear current plot
            plt.clf()

            # Plotting
            plt.subplot(3, 1, 1)
            plt.plot(time_data, mq135_data, label='MQ135 Value', color='blue')
            plt.title('MQ135 Sensor Readings Over Time')
            plt.ylabel('MQ135 Value')
            plt.grid()
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.plot(time_data, ammonia_data, label='Ammonia (NH₃) Concentration', color='green')
            plt.title('Ammonia Concentration Over Time')
            plt.ylabel('Concentration (ppm)')
            plt.grid()
            plt.legend()

            plt.subplot(3, 1, 3)
            plt.plot(time_data, co2_data, label='CO₂ Concentration', color='red')
            plt.title('CO₂ Concentration Over Time')
            plt.ylabel('Concentration (ppm)')
            plt.grid()
            plt.legend()

            plt.xlabel('Time (seconds)')

        except (ValueError, IndexError):
            #handle errors
            print("Data format error. Please check the output from the Arduino.")
    
#animation
ani = FuncAnimation(plt.gcf(), update, interval=1000)

plt.tight_layout()
plt.show()

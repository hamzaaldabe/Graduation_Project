import serial
import matplotlib.pyplot as plt

fig, ax = plt.subplots()


time_data = []
angle_data = []

# Initialize the serial connection
ser = serial.Serial('COM3', 9600)

while True:
    data = ser.readline().decode().strip().split(',')

    time = float(data[0])
    angle = float(data[1])
    
    time_data.append(time)
    angle_data.append(angle)
    
    ax.plot(time_data, angle_data)
    plt.pause(0.01)
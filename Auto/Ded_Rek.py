import numpy as np
import serial
import time
import matplotlib.pyplot as plt
from filterpy.kalman import ExtendedKalmanFilter as EKF

ser = serial.Serial('PORT', 115200)
time.sleep(2)
# EKF Initialization
dt = 0.1  # Time step (seconds)

# Initialize the EKF
ekf = EKF(dim_x=5, dim_z=5)  # State: [x, y, vx, vy, theta]

# Initial state
ekf.x = np.zeros((5, 1))  # Initial state vector [x, y, vx, vy, theta]

# Define the state transition matrix and process noise covariance
ekf.F = np.eye(5)  # Placeholder, will be updated in the predict function
ekf.Q = np.diag([0.1, 0.1, 0.01, 0.01, 0.1])  # Process noise covariance matrix

# Measurement noise covariance
ekf.R = np.diag([0.5, 0.5, 0.5, 0.5, 0.5])  # Measurement noise covariance matrix
ekf.H = np.eye(5)  # Measurement matrix

# Lists for plotting
positions = []
times = []

# State transition function
def state_transition(ekf, dt):
    # Update the state transition matrix F based on the current state
    theta = ekf.x[4, 0]
    ekf.F = np.array([[1, 0, dt * np.cos(theta), 0, 0],
                      [0, 1, 0, dt * np.sin(theta), 0],
                      [0, 0, 1, 0, 0],
                      [0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 1]])

# Real-time dead reckoning loop
try:
    while True:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        data = line.split(',')

        # Parse accelerometer and gyroscope data
        ax = int(data[0]) / 16384.0  # Scale to g
        ay = int(data[1]) / 16384.0
        az = int(data[2]) / 16384.0
        gx = int(data[3]) / 131.0  # Scale to degrees/s
        gy = int(data[4]) / 131.0
        gz = int(data[5]) / 131.0

        # Prediction step
        state_transition(ekf, dt)  # Update state transition matrix
        ekf.predict()

        # Create measurement vector [x, y, vx, vy, theta]
        z = np.array([[ekf.x[0, 0] + ax * dt**2 / 2],  # Position estimate based on acceleration
                      [ekf.x[1, 0] + ay * dt**2 / 2],
                      [gx],
                      [gy],
                      [az]])

        # Update step
        ekf.update(z)

        # Store data for plotting
        positions.append((ekf.x[0, 0], ekf.x[1, 0]))
        times.append(time.time())

        # Print the current position and orientation
        print(f"Position: {ekf.x[0:2].flatten()}, Orientation: {ekf.x[4, 0]}")
        time.sleep(dt)

except KeyboardInterrupt:
    print("Stopped.")

#numpy array for plotting
positions = np.array(positions)

plt.figure()
plt.plot(positions[:, 0], positions[:, 1])
plt.title('Dead Reckoning Posn: ')
plt.xlabel('X Posn (m): ')
plt.ylabel('Y Posn (m): ')
plt.axis('equal')
plt.grid()
plt.show()

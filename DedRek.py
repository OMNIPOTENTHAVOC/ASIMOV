import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
import time
import smbus2
import serial
import pynmea2
import matplotlib.pyplot as plt

#Kalman Filter
ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
ekf.x = np.array([0, 0, 0, 0])  # Initial state: [lon, vx, lat, vy]
ekf.P *= 1000  # Initial uncertainty
ekf.R = np.diag([0.0001, 0.0001])  # Measurement noise
ekf.Q = np.eye(4) * 0.1  # Process noise

est_pos = []

def f(state, dt):
    x, vx, y, vy = state
    return np.array([x + vx * dt, vx, y + vy * dt, vy])

def Jf(state, dt):
    return np.array([[1, dt, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, dt],
                     [0, 0, 0, 1]])

def h(state):
    x, _, y, _ = state
    return np.array([x, y])

def Jh(state):
    return np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0]])

#getting real-time GPS data
def get_gps_data():
    with serial.Serial('/dev/ttyUSB0', 9600, timeout=1) as ser:
        while True:
            line = ser.readline().decode('ascii', errors='replace')
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                msg = pynmea2.parse(line)
                if msg.latitude and msg.longitude:
                    return msg.longitude, msg.latitude

#getting real-time IMU data
MPU6050_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D

#I2C
bus = smbus2.SMBus(1)  # 1 for Raspberry Pi's default I2C bus

def read_word_2c(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    val = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

def get_imu_data():
    ax = read_word_2c(ACCEL_XOUT_H) / 16384.0  # Scale factor for MPU6050
    ay = read_word_2c(ACCEL_YOUT_H) / 16384.0
    return ax, ay

vx, vy, x, y = 0, 0, 0, 0  #init. vel and pos
last_time = time.time()

try:
    while True:
        #real-time GPS and IMU data
        gps_x, gps_y = get_gps_data()
        ax, ay = get_imu_data()

        #calculate delta time
        curr_time = time.time()
        dt = curr_time - last_time
        last_time = curr_time

        # Dedrek based on IMU
        dr_x, dr_y, dr_vx, dr_vy = x + vx * dt + 0.5 * ax * dt ** 2, \
                                   y + vy * dt + 0.5 * ay * dt ** 2, \
                                   vx + ax * dt, \
                                   vy + ay * dt

        # Kalman filter update
        ekf.F = Jf(ekf.x, dt)
        ekf.x = f(ekf.x, dt)
        ekf.predict()

        #GPS data for the measurement
        z = np.array([gps_x, gps_y])
        ekf.H = Jh(ekf.x)
        ekf.update(z, HJacobian=Jh, Hx=h)

        #pos and vel estimate updates
        x, vx, y, vy = ekf.x
        est_pos.append((y, x))
        print(f"Estimated Position: Latitude = {y:.6f}, Longitude = {x:.6f}")

        # Update dead reckoning for next iteration
        vx, vy, x, y = dr_vx, dr_vy, dr_x, dr_y

        time.sleep(0.1)  #rate adjustment

except KeyboardInterrupt:
    print("Process interrupted.")

# Plot the estimated path
lats, lons = zip(*est_pos)
plt.plot(lons, lats, marker='o', label='Estimated Path')
plt.title('Real-Time Path Estimation')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.legend()
plt.grid()
plt.show()

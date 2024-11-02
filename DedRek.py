
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
import time
import matplotlib.pyplot as plt


s_lat, s_lon = 34.0522, -118.2437  
e_lat, e_lon = 34.0525, -118.2430 

n_steps = 20  
lat_step = (e_lat - s_lat) / n_steps
lon_step = (e_lon - s_lon) / n_steps

ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
ekf.x = np.array([s_lon, 0, s_lat, 0])
ekf.P *= 1000
ekf.R = np.diag([0.0001, 0.0001])
ekf.Q = np.eye(4) * 0.1

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

def sim_gps(step):
    gps_lon = s_lon + lon_step * step
    gps_lat = s_lat + lat_step * step
    
    
    gps_lon += np.random.normal(0, 0.00005)
    gps_lat += np.random.normal(0, 0.00005)
    
    return gps_lon, gps_lat

def sim_mpu():
   
    ax_noise = np.random.normal(0, 0.01)
    ay_noise = np.random.normal(0, 0.01)
    return ax_noise, ay_noise

def dr(ax, ay, dt, vx, vy, x, y):
    vx += ax * dt
    vy += ay * dt
    x += vx * dt + 0.5 * ax * dt ** 2
    y += vy * dt + 0.5 * ay * dt ** 2
    return x, y, vx, vy

vx, vy, x, y = 0, 0, s_lon, s_lat
last_time = time.time()

for step in range(n_steps):
    gps_x, gps_y = sim_gps(step)
    ax, ay = sim_mpu()
    
    curr_time = time.time()
    dt = curr_time - last_time
    last_time = curr_time
    
    dr_x, dr_y, dr_vx, dr_vy = dr(ax, ay, dt, vx, vy, x, y)
    
    ekf.F = Jf(ekf.x, dt)
    ekf.x = f(ekf.x, dt)
    ekf.predict()

    z = np.array([gps_x, gps_y])
    ekf.H = Jh(ekf.x)
    ekf.update(z, HJacobian=Jh, Hx=h)
    
    x, vx, y, vy = ekf.x
    est_pos.append((y, x))
    print(f"Estimated Position: Latitude = {y:.6f}, Longitude = {x:.6f}")
    
    vx, vy, x, y = dr_vx, dr_vy, dr_x, dr_y
    
    time.sleep(0.1)

print("Goal reached!")


# Plot the estimated path with noise
lats, lons = zip(*est_pos)
plt.figure()
plt.plot(lons, lats, marker='o', label='Estimated Path with Noise')
plt.plot(s_lon, s_lat, 'go', label='Start')
plt.plot(e_lon, e_lat, 'ro', label='End')
plt.plot([s_lon, e_lon], [s_lat, e_lat], 'b--', label='Straight Line')
plt.title('Path from Start to Goal with Noise')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.legend()
plt.grid()

# Plot from dead reckoning
ideal_lons = [s_lon + lon_step * step for step in range(n_steps)]
ideal_lats = [s_lat + lat_step * step for step in range(n_steps)]
plt.figure()
plt.plot(ideal_lons, ideal_lats, marker='o', linestyle='-', color='green', label='Corrected Path')
plt.plot(s_lon, s_lat, 'go', label='Start')
plt.plot(e_lon, e_lat, 'ro', label='End')
plt.plot([s_lon, e_lon], [s_lat, e_lat], 'b--', label='Actual Path')
plt.title('Path Based on Dead Reckoning')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.legend()
plt.grid()

plt.show()

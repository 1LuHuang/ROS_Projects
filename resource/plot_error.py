import pandas as pd
import matplotlib.pyplot as plt
import math

csv_path = '/home/yiluhuang/workspace/Desktop/ros2_ws/trajectory_error_log.csv'

df = pd.read_csv(csv_path)

e = df['cross_track_error']

rmse = math.sqrt((e ** 2).mean())
mean_error = e.mean()
max_error = e.max()

print('Trajectory Tracking Evaluation')
print(f'RMSE:       {rmse:.4f} m')
print(f'Mean Error: {mean_error:.4f} m')
print(f'Max Error:  {max_error:.4f} m')

plt.figure()
plt.plot(df['time'], df['cross_track_error'])
plt.xlabel('Time [s]')
plt.ylabel('Cross-track Error [m]')
plt.title('Cross-track Error over Time')
plt.grid(True)
plt.savefig('cross_track_error.png', dpi=300)
plt.show()

plt.figure()
plt.plot(df['x'], df['y'], label='Actual Path')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Actual Vehicle Trajectory')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.savefig('actual_trajectory.png', dpi=300)
plt.show()
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Load the simulation data
df = pd.read_csv('../phase1_dynamics.csv')

fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

# 1. Plot the 3D Flight Path
ax.plot(df['x'], df['y'], df['z'], label='Flight Path', color='royalblue', linewidth=2)

# 2. Draw a 3D Drone Model at the Final Position
# Get final state
x_f, y_f, z_f = df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1]
yaw, pitch, roll = df['yaw'].iloc[-1], df['pitch'].iloc[-1], df['roll'].iloc[-1]

# Rotation matrix from Euler angles (ZYX convention)
R = np.array([
    [np.cos(yaw)*np.cos(pitch), np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.sin(yaw)*np.cos(roll), np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll)],
    [np.sin(yaw)*np.cos(pitch), np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(roll), np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(roll)],
    [-np.sin(pitch), np.cos(pitch)*np.sin(roll), np.cos(pitch)*np.cos(roll)]
])

# Drone arm length (visual scale)
L = 0.5
# Base "X" coordinates for the 4 arms
arms = np.array([
    [ L,  L, 0], [-L,  L, 0], [-L, -L, 0], [ L, -L, 0]
])

# Rotate the arms
rotated_arms = (R @ arms.T).T

# Plot the arms
for i in range(4):
    ax.plot([x_f, x_f + rotated_arms[i, 0]], 
            [y_f, y_f + rotated_arms[i, 1]], 
            [z_f, z_f + rotated_arms[i, 2]], 
            color='red', linewidth=3, solid_capstyle='round')

# Plot the center body
ax.scatter([x_f], [y_f], [z_f], color='black', s=100, zorder=10)

# 3. Formatting
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Phase 1 Milestone: 6-DOF Hover & Yaw Spin', fontsize=14, fontweight='bold')
ax.legend()

# Set equal aspect ratio
max_range = 2.0
ax.set_xlim(x_f - max_range, x_f + max_range)
ax.set_ylim(y_f - max_range, y_f + max_range)
ax.set_zlim(0, 6)

# Save the plot
plt.tight_layout()
plt.savefig('../docs/phase1_3d_trajectory.png', dpi=300)
print("3D Plot saved to docs/phase1_3d_trajectory.png")

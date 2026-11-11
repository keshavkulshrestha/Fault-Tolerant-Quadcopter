import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('../phase3_waypoint_wind.csv')

fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
fig.suptitle('Phase 3 Milestone: Waypoint Tracking & Wind Gust Rejection', fontsize=14, fontweight='bold')

# Plot 1: X Axis Tracking
axs[0].plot(df['time'], df['target_x'], 'r--', label='Target X')
axs[0].plot(df['time'], df['actual_x'], 'b-', label='Actual X')
axs[0].set_ylabel('X Position (m)')
axs[0].legend(loc='lower right')
axs[0].grid(True)

# Plot 2: Y Axis Tracking
axs[1].plot(df['time'], df['target_y'], 'r--', label='Target Y')
axs[1].plot(df['time'], df['actual_y'], 'g-', label='Actual Y')
axs[1].set_ylabel('Y Position (m)')
axs[1].legend(loc='lower right')
axs[1].grid(True)

# Plot 3: Wind Disturbance
axs[2].plot(df['time'], df['wind_x'], 'm-', label='Wind Force (N)')
axs[2].set_ylabel('Wind (N)')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylim(-1, 6)
axs[2].legend(loc='upper right')
axs[2].grid(True)

# Highlight the wind gust period
axs[0].axvspan(3.0, 4.0, color='red', alpha=0.1, label='Wind Gust')
axs[1].axvspan(3.0, 4.0, color='red', alpha=0.1)
axs[2].axvspan(3.0, 4.0, color='red', alpha=0.1)

plt.tight_layout()
plt.savefig('../docs/phase3_wind_rejection.png', dpi=300)
print("Wind rejection plot saved to docs/phase3_wind_rejection.png")

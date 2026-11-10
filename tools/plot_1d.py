import pandas as pd
import matplotlib.pyplot as plt

# Load the data (go up one directory to find the CSV)
df = pd.read_csv('../simulation_data.csv')

fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
fig.suptitle('Phase 0 Milestone: 1D PID Altitude Hold', fontsize=16)

# Plot Altitude
axs[0].plot(df['time'], df['z'], label='Altitude (z)', color='blue')
axs[0].axhline(5.0, color='r', linestyle='--', label='Target (5m)')
axs[0].set_ylabel('Altitude (m)')
axs[0].legend()
axs[0].grid(True)

# Plot Velocity
axs[1].plot(df['time'], df['vz'], label='Velocity (vz)', color='orange')
axs[1].set_ylabel('Velocity (m/s)')
axs[1].legend()
axs[1].grid(True)

# Plot Thrust
axs[2].plot(df['time'], df['thrust'], label='Thrust Command', color='green')
axs[2].axhline(9.81, color='gray', linestyle=':', label='Hover Thrust (mg)')
axs[2].set_ylabel('Thrust (N)')
axs[2].set_xlabel('Time (s)')
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
plt.savefig('../docs/phase0_milestone.png', dpi=300)
plt.show()
print("Plot saved to docs/phase0_milestone.png")

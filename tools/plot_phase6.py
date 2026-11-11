import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('../phase6_recovery.csv')

fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
fig.suptitle('Phase 6 Milestone: Fault Tolerance and Controlled Recovery', fontsize=14, fontweight='bold')

# Plot 1: Motor Forces & State
axs[0].plot(df['time'], df['m1'], 'r-', label='Motor 1 (Failed)')
axs[0].plot(df['time'], df['m2'], 'g-', label='Motor 2')
axs[0].plot(df['time'], df['m3'], 'b-', label='Motor 3')
axs[0].plot(df['time'], df['m4'], 'm-', label='Motor 4')
axs[0].set_ylabel('Force (N)')
axs[0].legend(loc='upper right')
axs[0].grid(True)

# Plot 2: Altitude Tracking
axs[1].plot(df['time'], df['target_z'], 'k--', label='Target Altitude')
axs[1].plot(df['time'], df['z'], 'b-', linewidth=2, label='Actual Altitude')
axs[1].set_ylabel('Altitude (m)')
axs[1].legend(loc='upper right')
axs[1].grid(True)

# Plot 3: State Machine
axs[2].plot(df['time'], df['state'], 'r-', linewidth=2, label='Flight State (0=Normal, 1=Recovery, 2=Landed)')
axs[2].set_ylabel('State')
axs[2].set_xlabel('Time (s)')
axs[2].set_yticks([0, 1, 2])
axs[2].set_yticklabels(['NORMAL', 'RECOVERY', 'LANDED'])
axs[2].legend(loc='upper right')
axs[2].grid(True)

# Highlight fault injection
for ax in axs:
    ax.axvline(3.0, color='black', linestyle=':', linewidth=2, label='Fault Injection')

plt.tight_layout()
plt.savefig('../docs/phase6_recovery.png', dpi=300)
print("Recovery plot saved to docs/phase6_recovery.png")

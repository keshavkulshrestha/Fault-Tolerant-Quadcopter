import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('../phase5_fdi.csv')

fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle('Phase 5 Milestone: Fault Detection & Isolation (Motor 1 Failure)', fontsize=14, fontweight='bold')

# Plot 1: Motor Forces
axs[0].plot(df['time'], df['m1'], 'r-', label='Motor 1 (Failed)')
axs[0].plot(df['time'], df['m2'], 'g-', label='Motor 2')
axs[0].plot(df['time'], df['m3'], 'b-', label='Motor 3')
axs[0].plot(df['time'], df['m4'], 'm-', label='Motor 4')
axs[0].set_ylabel('Force (N)')
axs[0].legend(loc='upper right')
axs[0].grid(True)

# Plot 2: Roll Rate (The physical effect of the fault)
axs[1].plot(df['time'], df['roll_rate'], 'k-', linewidth=2, label='Measured Roll Rate')
axs[1].set_ylabel('Roll Rate (rad/s)')
axs[1].legend(loc='upper right')
axs[1].grid(True)

# Plot 3: Residual & Fault State
axs[2].plot(df['time'], df['residual'], 'r-', linewidth=2, label='Dynamic Residual')
axs[2].axhline(15.0, color='red', linestyle='--', label='Fault Threshold')
axs[2].fill_between(df['time'], 0, 20, where=(df['fault_state']==1), color='red', alpha=0.2, label='FAULT STATE')
axs[2].set_ylabel('Residual')
axs[2].set_xlabel('Time (s)')
axs[2].legend(loc='upper right')
axs[2].grid(True)

# Highlight the fault injection time
for ax in axs:
    ax.axvline(3.0, color='black', linestyle=':', linewidth=2, label='Fault Injection')

plt.tight_layout()
plt.savefig('../docs/phase5_fdi.png', dpi=300)
print("FDI plot saved to docs/phase5_fdi.png")

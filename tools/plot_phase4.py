import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('../phase4_estimation.csv')

plt.figure(figsize=(12, 6))
plt.plot(df['time'], df['true_roll'], 'k-', linewidth=2, label='True Roll (Ground Truth)')
plt.plot(df['time'], df['noisy_accel_roll'], 'r.', markersize=2, label='Noisy Accelerometer (Raw Angle)', alpha=0.5)
plt.plot(df['time'], df['filtered_roll'], 'b-', linewidth=2, label='Complementary Filter Output')

plt.title('Phase 4 Milestone: Sensor Fusion & Complementary Filter', fontsize=14, fontweight='bold')
plt.xlabel('Time (s)')
plt.ylabel('Roll Angle (rad)')
plt.legend(loc='upper right')
plt.grid(True)
plt.tight_layout()

plt.savefig('../docs/phase4_sensor_fusion.png', dpi=300)
print("Sensor fusion plot saved to docs/phase4_sensor_fusion.png")

import pandas as pd
import matplotlib.pyplot as plt

# Read CSV into a DataFrame
df = pd.read_csv('output.csv')
ap = max(df['Alt'])

fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Top plot (subplot 1)
axs[0].plot(df['Time'], df['Accel_noisy'], label='Acceleration (raw)')
axs[0].plot(df['Time'], df['Accel_kalman'], label='Acceleration (filtered)')
axs[0].plot(df['Time'], df['Accel_real'], label='Acceleration (real)')
axs[0].plot(df['Time'], df['Velocity'], label='Velocity')
axs[0].plot(df['Time'], df['Alt'], label='Alt')
axs[0].plot(df['Time'], df['Alt_real'], label='Alt_real')

# axs[0].plot(df['Time'], df['Ap_pred'], label='Predicted apogee')
non_zero_indices = df['Ap_pred'] != 0
df_filtered = df[non_zero_indices]
axs[0].plot(df_filtered['Time'], df_filtered['Ap_pred'], label='Predicted apogee')
axs[0].axhline(y=0, color='black')
axs[0].axhline(y=ap, color='red', linestyle='--', label='Apogee: {}m'.format(int(ap)))
axs[0].set_ylim(-3*ap/2, 3*ap/2)
axs[0].legend()

# Bottom plot (subplot 2)
axs[1].plot(df['Time'], df['A'], label='Servo Signal')
axs[1].legend()

# Set common labels and show the plot
plt.xlabel('Time')
plt.tight_layout()
plt.show()

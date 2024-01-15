import pandas as pd
import matplotlib.pyplot as plt

# Read CSV into a DataFrame
df = pd.read_csv('output.csv')



# Top plot (subplot 1)
plt.plot(df['Time'], df['Accel_noisy'], label='Acceleration (raw)')
plt.plot(df['Time'], df['Accel_kalman'], label='Acceleration (filtered)')
plt.plot(df['Time'], df['Accel_real'], label='Acceleration (real)')
plt.axhline(y=0, color='black')
plt.legend()

# Set common labels and show the plot
plt.xlabel('Time')
plt.tight_layout()
plt.show()

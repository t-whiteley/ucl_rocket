import pandas as pd
import matplotlib.pyplot as plt

# Read CSV into a DataFrame
df = pd.read_csv('output.csv')
ap = max(df['Altitude'])

# Plot the data
plt.plot(df['Time'], df['Acceleration'], label='Acceleration')
plt.plot(df['Time'], df['Velocity'], label='Velocity')
plt.plot(df['Time'], df['Altitude'], label='Altitude')

# Add labels and legend
plt.xlabel('Time')
plt.ylabel('Value')
plt.axhline(y=0, color='black')
plt.axhline(y=ap, color='red', linestyle='--', label='Apogee: {}m'.format(int(ap)))
plt.legend()

# Show the plot
plt.show()
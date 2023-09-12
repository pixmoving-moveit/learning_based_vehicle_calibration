import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


fig = plt.figure()
ax = plt.axes(projection='3d')




columns = ["Velocity", "Throttling", "Acceleration_measured"]
df = pd.read_csv("throttling2.csv", usecols=columns)

xdata = df.Velocity
ydata = df.Throttling
zdata = df.Acceleration_measured


cmap = plt.get_cmap('Greens')  # You can choose any colormap you like
normalize = plt.Normalize(zdata.min(), zdata.max())

# Create a ScalarMappable to map values to colors
sc = plt.cm.ScalarMappable(cmap=cmap, norm=normalize)
sc.set_array([])

# Scatter plot with colors based on Acceleration_measured values
scatter = ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap=cmap, marker='o')

# Add color bar to the plot for reference
cbar = plt.colorbar(sc, ax=ax, label='Acceleration_measured')

plt.show()


#plt.savefig('throttle_calibration_table.png')

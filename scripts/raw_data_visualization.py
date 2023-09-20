import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.linear_model import LinearRegression

# Read the data
columns = ["Velocity", "Braking", "Acceleration_with_pitch_comp"]
df = pd.read_csv("braking_testing.csv", usecols=columns)

xdata = df.Velocity
ydata = df.Braking
zdata = df.Acceleration_with_pitch_comp

# Fit a linear regression model
X = np.column_stack((xdata, ydata))
model = LinearRegression().fit(X, zdata)

# Create a grid of points in the x-y plane
x_grid, y_grid = np.meshgrid(np.linspace(xdata.min(), xdata.max(), 100),
                             np.linspace(ydata.min(), ydata.max(), 100))

# Compute the corresponding z-values using the plane equation
z_grid = model.predict(np.column_stack((x_grid.ravel(), y_grid.ravel())))
z_grid = z_grid.reshape(x_grid.shape)

# Create a 3D figure
fig = plt.figure()
ax = plt.axes(projection='3d')

# Scatter plot with colors based on Acceleration_measured values
cmap = plt.get_cmap('Greens')  # You can choose any colormap you like
normalize = plt.Normalize(zdata.min(), zdata.max())
sc = plt.cm.ScalarMappable(cmap=cmap, norm=normalize)
sc.set_array([])

scatter = ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap=cmap, marker='o')

# Plot the plane
ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.5, cmap=cmap)

# Customize the plot
ax.set_xlabel('Velocity')
ax.set_zlabel('Acceleration')
ax.set_ylabel('Braking Output')


# Add color bar to the plot for reference
cbar = plt.colorbar(sc, ax=ax, label='Acceleration_with_pitch_comp')

plt.show()






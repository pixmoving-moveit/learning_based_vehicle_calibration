import os
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import medfilt
from sklearn.metrics import mean_squared_error
from sklearn.metrics import mean_absolute_error
from sklearn.metrics import r2_score


# Load the CSV file into a DataFrame
data = pd.read_csv('throttling_robobus_final.csv')

# Standardize data
threshold0 = 1.9
threshold1 = 2
threshold2 = 1.8

mean0 = data["Velocity"].mean()
std0 = data["Velocity"].std()
data["Velocity"] = (data["Velocity"] - mean0) / std0

data = data[abs(data["Velocity"]-mean0) <= std0*threshold0]


mean1 = data["Throttling"].mean()
std1 = data["Throttling"].std()
data["Throttling"] = (data["Throttling"] - mean1) / std1

data = data[abs(data["Throttling"]-mean1) <= std1*threshold1]

mean2 = data["Acceleration_with_pitch_comp"].mean()
std2 = data["Acceleration_with_pitch_comp"].std()
data["Acceleration_with_pitch_comp"] = (data["Acceleration_with_pitch_comp"] - mean2) / std2

data = data[abs(data["Acceleration_with_pitch_comp"]-mean2) <= std2*threshold2]


# Split the data into input features (velocity and acceleration) and target (command)

X = data[['Velocity', 'Throttling']].values
y = data['Acceleration_with_pitch_comp'].values



# Split the data into training and testing sets (e.g., 80% train, 20% test)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)



# Convert NumPy arrays to PyTorch tensors
X_train = torch.tensor(X_train, dtype=torch.float32)
y_train = torch.tensor(y_train, dtype=torch.float32)
X_test = torch.tensor(X_test, dtype=torch.float32)
y_test = torch.tensor(y_test, dtype=torch.float32)




# Define the same custom model class
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.fc1 = nn.Linear(2, 128)
        self.sigmoid1 = nn.Sigmoid()
        self.fc2 = nn.Linear(128, 64)
        self.sigmoid2 = nn.Sigmoid()
        self.fc3 = nn.Linear(64, 1)

    def forward(self, x):
        x = self.fc1(x)
        x = self.sigmoid1(x)
        x = self.fc2(x)
        x = self.sigmoid2(x)
        x = self.fc3(x)
        return x

# Create an instance of the model
model = NeuralNetwork()

# Load the trained model's state dict
model.load_state_dict(torch.load('trained_throttle_model.pth'))

# Put the model in evaluation mode (important for dropout and batch normalization layers)
model.eval()



# Make predictions on new data
new_data = np.array([[(4-mean0)/std0, (7-mean1)/std1], [(0.5-mean0)/std0, (50-mean1)/std1]])  # Example new data with velocity and acceleration
#new_data = scaler.transform(new_data)  # Normalize the new data
new_data = torch.tensor(new_data, dtype=torch.float32)
with torch.no_grad():
    predictions = model(new_data)*std2+mean2
    print("Predicted Commands for New Data:")
    for i, pred in enumerate(predictions):
        print(f"Data {i + 1}: {pred.item()}")

        

# Visualization
# Create a meshgrid for velocity and acceleration
#velocity_range = np.linspace((X[:, 0]*std0+mean0).min(), (X[:, 0]*std0+mean0).max(), 100)
#throttling_range = np.linspace((X[:, 1]*std1+mean1).min(), (X[:, 1]*std1+mean1).max(), 100)
velocity_range = np.linspace(0, 9.17, 100)
throttling_range = np.linspace(0, (X[:, 1]*std1+mean1).max(), 100)
V, A = np.meshgrid(velocity_range, throttling_range)

# Create a grid of input data points for prediction
input_grid = np.column_stack(((V.flatten()-mean0)/std0, (A.flatten()-mean1)/std1))
input_grid = torch.tensor(input_grid, dtype=torch.float32)

# Predict the throttling commands for the entire input grid
with torch.no_grad():
    commands = model(input_grid).reshape(V.shape)
    
    
commands_new = commands*std2+mean2
    

# Evaluate the model on the test data
with torch.no_grad():
    test_outputs = model(X_test)
    


# Calculate MSE
mse = mean_squared_error(y_test, test_outputs.view(-1).numpy())
print(f"Mean Squared Error on Test Data: {mse}")


# Calculate MAE
mae = mean_absolute_error(y_test, test_outputs.view(-1).numpy())
print(f"Mean Absolute Error on Test Data: {mae}")


rmse = np.sqrt(mse)
print(f"Root Mean Squared Error on Test Data: {rmse}")


# Calculate R2 score
r2 = r2_score(y_test, test_outputs.view(-1).numpy())
print(f"R-squared (R2) Score on Test Data: {r2}")





# Create headers for velocity values
velocity_headers = ['{:.2f}'.format(v) for v in velocity_range]

# Create headers for throttling values
throttling_range /= 100
throttling_headers = ['Throttling {:.2f}'.format(a) for a in throttling_range]

# Combine headers for the first row of the CSV
headers = [''] + velocity_headers

# Add throttling values to the commands_new matrix as the first column
commands_new_with_throttling = np.column_stack((throttling_range, commands_new))

# Save the commands_new matrix along with headers to a CSV file
csv_filename = 'commands_throttle.csv'
np.savetxt(csv_filename, commands_new_with_throttling, delimiter=',', header=','.join(headers), comments='')





# Create the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
surf = ax.plot_surface(V, A, commands_new, cmap='viridis')

# Customize the plot
ax.set_xlabel('Velocity')
ax.set_zlabel('Acceleration')
ax.set_ylabel('Throttling Output')
ax.set_title('Neural Network Output vs. Velocity and Throttling')

# Add a color bar which maps values to colors
fig.colorbar(surf)

# Show the plot
plt.show()

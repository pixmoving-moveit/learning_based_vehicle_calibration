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

# Load the CSV file into a DataFrame
data = pd.read_csv('braking_bag_9_15.csv')

# Standardize data

mean0 = data["Velocity"].mean()
std0 = data["Velocity"].std()
data["Velocity"] = (data["Velocity"] - mean0) / std0

mean1 = data["Braking"].mean()
std1 = data["Braking"].std()
data["Braking"] = (data["Braking"] - mean1) / std1

mean2 = data["Acceleration_with_pitch_comp"].mean()
std2 = data["Acceleration_with_pitch_comp"].std()
data["Acceleration_with_pitch_comp"] = (data["Acceleration_with_pitch_comp"] - mean2) / std2


# Split the data into input features (velocity and acceleration) and target (command)

X = data[['Velocity', 'Braking']].values
y = data['Acceleration_with_pitch_comp'].values



# Split the data into training and testing sets (e.g., 80% train, 20% test)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)



# Convert NumPy arrays to PyTorch tensors
X_train = torch.tensor(X_train, dtype=torch.float32)
y_train = torch.tensor(y_train, dtype=torch.float32)
X_test = torch.tensor(X_test, dtype=torch.float32)
y_test = torch.tensor(y_test, dtype=torch.float32)

# Define a custom PyTorch model

class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.fc1 = nn.Linear(2, 128)  # Input layer with 2 neurons, hidden layer with n neurons
        self.sigmoid1 = nn.Sigmoid()
        self.fc2 = nn.Linear(128, 32)
        self.sigmoid2 = nn.Sigmoid()
        self.fc3 = nn.Linear(32, 1)  # Output layer with 1 neuron

    def forward(self, x):
        x = self.fc1(x)
        x = self.sigmoid1(x)
        x = self.fc2(x)
        x = self.sigmoid2(x)
        x = self.fc3(x)
        return x

# Create an instance of the model
model = NeuralNetwork()

# Define a loss function and optimizer
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Training loop
num_epochs = 100
for epoch in range(num_epochs):
    # Forward pass
    outputs = model(X_train)
    
    loss = criterion(outputs, y_train.view(-1, 1))  # Calculate the loss

    # Backpropagation and optimization
    optimizer.zero_grad()  # Zero the gradients
    loss.backward()  # Backpropagate the gradients
    optimizer.step()  # Update the weights

# Evaluate the model on the test data
with torch.no_grad():
    test_outputs = model(X_test)
    test_loss = criterion(test_outputs, y_test.view(-1, 1))
    print(f"Mean Squared Error on Test Data: {test_loss.item()}")

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
velocity_range = np.linspace((X[:, 0]*std0+mean0).min(), (X[:, 0]*std0+mean0).max(), 100)
braking_range = np.linspace((X[:, 1]*std1+mean1).min(), (X[:, 1]*std1+mean1).max(), 100)
V, A = np.meshgrid(velocity_range, braking_range)

# Create a grid of input data points for prediction
input_grid = np.column_stack(((V.flatten()-mean0)/std0, (A.flatten()-mean1)/std1))
input_grid = torch.tensor(input_grid, dtype=torch.float32)

# Predict the throttling commands for the entire input grid
with torch.no_grad():
    commands = model(input_grid).reshape(V.shape)
    
    
commands_new = commands*std2+mean2
    


# Create the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the surface
surf = ax.plot_surface(V, A, commands_new, cmap='viridis')

# Customize the plot
ax.set_xlabel('Velocity')
ax.set_zlabel('Acceleration')
ax.set_ylabel('Braking Output')
ax.set_title('Neural Network Output vs. Velocity and Braking')

# Add a color bar which maps values to colors
fig.colorbar(surf)

# Show the plot
plt.show()





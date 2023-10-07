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


data = pd.read_csv('throttling_robobus_final.csv')
dataa = pd.read_csv('throttling_robobus_final.csv')

# Standardize data and remove outliers according to thresholds
threshold0 = 1.9
threshold1 = 2
threshold2 = 1.8
threshold3 = 2

mean0 = data["Velocity"].mean()
std0 = data["Velocity"].std()
data["Velocity"] = (data["Velocity"] - mean0) / std0
dataa["Velocity"] = (dataa["Velocity"] - mean0) / std0

data = data[abs(data["Velocity"]-mean0) <= std0*threshold0]
dataa = dataa[abs(dataa["Velocity"]-mean0) <= std0*threshold0]

mean1 = data["Throttling"].mean()
std1 = data["Throttling"].std()
data["Throttling"] = (data["Throttling"] - mean1) / std1
dataa["Throttling"] = (dataa["Throttling"] - mean1) / std1

data = data[abs(data["Throttling"]-mean1) <= std1*threshold1]
dataa = dataa[abs(dataa["Throttling"]-mean1) <= std1*threshold1]

mean2 = data["Acceleration_with_pitch_comp"].mean()
std2 = data["Acceleration_with_pitch_comp"].std()
data["Acceleration_with_pitch_comp"] = (data["Acceleration_with_pitch_comp"] - mean2) / std2
dataa["Acceleration_with_pitch_comp"] = (dataa["Acceleration_with_pitch_comp"] - mean2) / std2

data = data[abs(data["Acceleration_with_pitch_comp"]-mean2) <= std2*threshold2]
dataa = dataa[abs(dataa["Acceleration_with_pitch_comp"]-mean2) <= std2*threshold2]

mean3 = data["Steering"].mean()
std3 = data["Steering"].std()
data["Steering"] = (data["Steering"] - mean3) / std3
dataa["Steering"] = (dataa["Steering"] - mean3) / std3

data = data[abs(data["Steering"] - mean3) <= std3*threshold3]
dataa = dataa[abs(dataa["Steering"]-mean3) <= std3*threshold3]


# Split the data into input features (velocity and throttle) and target (acceleration) and test/train

X = data[['Velocity', 'Throttling', 'Steering']].values
y = data['Acceleration_with_pitch_comp'].values


X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)



# Convert NumPy arrays to PyTorch tensors
X_train = torch.tensor(X_train, dtype=torch.float32)
y_train = torch.tensor(y_train, dtype=torch.float32)
X_test = torch.tensor(X_test, dtype=torch.float32)
y_test = torch.tensor(y_test, dtype=torch.float32)


# NN model

class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.fc1 = nn.Linear(3, 128)  # Input layer with 3 neurons, hidden layer with n neurons
        self.sigmoid1 = nn.Sigmoid()
        self.fc2 = nn.Linear(128, 64)
        self.sigmoid2 = nn.Sigmoid()
        self.fc3 = nn.Linear(64, 1)  # Output layer with 1 neuron
        

    def forward(self, x):
        x = self.fc1(x)
        x = self.sigmoid1(x)
        x = self.fc2(x)
        x = self.sigmoid2(x)
        x = self.fc3(x)
        return x


model = NeuralNetwork()

criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)


# Training loop
num_epochs = 100
for epoch in range(num_epochs):
    # Forward pass
    outputs = model(X_train)
    
    loss = criterion(outputs, y_train.view(-1, 1))  

    # Backpropagation and optimization
    optimizer.zero_grad()  
    loss.backward()  
    optimizer.step() 


with torch.no_grad():
    test_outputs = model(X_test)
    test_loss = criterion(test_outputs, y_test.view(-1, 1))
    #print(f"Mean Squared Error on Test Data: {test_loss.item()}")


# Example: make predictions on new data
new_data = np.array([[(4-mean0)/std0, (7-mean1)/std1, (10-mean3)/std3], [(0.5-mean0)/std0, (50-mean1)/std1, (40-mean3)/std3]]) 
#new_data = scaler.transform(new_data)  # Normalize the new data
new_data = torch.tensor(new_data, dtype=torch.float32)
with torch.no_grad():
    predictions = model(new_data)*std2+mean2
    print("Predicted Commands for New Data:")
    for i, pred in enumerate(predictions):
        print(f"Data {i + 1}: {pred.item()}")


# Visualization

velocity_range = np.linspace((X[:, 0]*std0+mean0).min(), (X[:, 0]*std0+mean0).max(), 100)
throttling_range = np.linspace((X[:, 1]*std1+mean1).min(), (X[:, 1]*std1+mean1).max(), 100)
steering_range = np.linspace((X[:, 2]*std3+mean3).min(),(X[:, 2]*std3+mean3).max(), 100)
V, A, S = np.meshgrid(velocity_range, throttling_range, steering_range)

input_grid = np.column_stack(((V.flatten()-mean0)/std0, (A.flatten()-mean1)/std1, (S.flatten()-mean3/std3)))
input_grid = torch.tensor(input_grid, dtype=torch.float32)

with torch.no_grad():
    commands = model(input_grid).reshape(V.shape)
    
    
commands_new = commands*std2+mean2



# Save the trained model
torch.save(model.state_dict(), 'trained_throttle_model.pth')


# evaluation
mse = mean_squared_error(y_test, test_outputs.view(-1).numpy())
print(f"Mean Squared Error on Test Data: {mse}")

mae = mean_absolute_error(y_test, test_outputs.view(-1).numpy())
print(f"Mean Absolute Error on Test Data: {mae}")

rmse = np.sqrt(mse)
print(f"Root Mean Squared Error on Test Data: {rmse}")

r2 = r2_score(y_test, test_outputs.view(-1).numpy())
print(f"R-squared (R2) Score on Test Data: {r2}")
    

xdata = dataa.Velocity*std0+mean0
ydata = dataa.Throttling*std1+mean1
zdata = dataa.Acceleration_with_pitch_comp*std2+mean2
kdata = dataa.Steering*std3+mean3



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

scatter = ax.scatter3D(xdata, ydata, zdata, c=kdata, marker='o')
surf = ax.plot_surface(V, A, S, commands_new, cmap='viridis')

ax.set_xlabel('Velocity')
ax.set_zlabel('Acceleration')
ax.set_ylabel('Throttling Output')
ax.set_title('Neural Network Output vs. Velocity and Throttling')

fig.colorbar(surf)

plt.show()

import os
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
from sklearn.metrics import mean_absolute_error
from sklearn.metrics import r2_score



data = pd.read_csv('throttling.csv')

# Standardize data and remove outliers according to the thresholds
threshold0 = 10
threshold1 = 10
threshold2 = 5

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


# Split the data into input features (velocity and command) and target (acceleration) and between training and testing sets

X = data[['Velocity', 'Throttling']].values
y = data['Acceleration_with_pitch_comp'].values


X = torch.tensor(X, dtype=torch.float32)
y = torch.tensor(y, dtype=torch.float32)




# NN model
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.fc1 = nn.Linear(2, 128)  # Input layer with 2 neurons, hidden layer with n neurons
        self.relu1 = nn.ReLU()
        self.fc2 = nn.Linear(128, 32)
        self.relu2 = nn.ReLU()
        self.fc3 = nn.Linear(32, 1)  # Output layer with 1 neuron
        
        
        
    def forward(self, x):
        x = self.fc1(x)
        x = self.relu1(x)
        x = self.fc2(x)
        x = self.relu2(x)
        x = self.fc3(x)
        
        
        return x


model = NeuralNetwork()

model.load_state_dict(torch.load('trained_throttle.pth'))

model.eval()



# Example: make predictions on new data
new_data = np.array([[(4-mean0)/std0, (7-mean1)/std1], [(0.5-mean0)/std0, (50-mean1)/std1]]) 
#new_data = scaler.transform(new_data)  # Normalize the new data
new_data = torch.tensor(new_data, dtype=torch.float32)
with torch.no_grad():
    predictions = model(new_data)*std2+mean2
    print("Predicted Commands for New Data:")
    for i, pred in enumerate(predictions):
        print(f"Data {i + 1}: {pred.item()}")

        

# Visualization

#velocity_range = np.linspace((X[:, 0]*std0+mean0).min(), (X[:, 0]*std0+mean0).max(), 20)
#throttling_range = np.linspace((X[:, 1]*std1+mean1).min(), (X[:, 1]*std1+mean1).max(), 20)

velocity_range = np.linspace(0, (X[:, 0]*std0+mean0).max(), 20)
throttling_range = np.linspace(0, (X[:, 1]*std1+mean1).max(), 20)
V, A = np.meshgrid(velocity_range, throttling_range)

input_grid = np.column_stack(((V.flatten()-mean0)/std0, (A.flatten()-mean1)/std1))
input_grid = torch.tensor(input_grid, dtype=torch.float32)

with torch.no_grad():
    commands = model(input_grid).reshape(V.shape)
    
    
commands_new = commands*std2+mean2
    

# Evaluate the model on the test data
with torch.no_grad():
    test_outputs = model(X)
    

mse = mean_squared_error(y, test_outputs.view(-1).numpy())
print(f"Mean Squared Error on Test Data: {mse}")

mae = mean_absolute_error(y, test_outputs.view(-1).numpy())
print(f"Mean Absolute Error on Test Data: {mae}")

rmse = np.sqrt(mse)
print(f"Root Mean Squared Error on Test Data: {rmse}")

r2 = r2_score(y, test_outputs.view(-1).numpy())
print(f"R-squared (R2) Score on Test Data: {r2}")





# Save NN model in csv correct format for testing in the real vehicle

velocity_headers = ['{:.2f}'.format(v) for v in velocity_range]

# we normalize throttling values between 0 and 1
throttling_range /= 100
throttling_headers = ['Throttling {:.2f}'.format(a) for a in throttling_range]

headers = [''] + velocity_headers

commands_new_with_throttling = np.column_stack((throttling_range, commands_new))

csv_filename = 'accel_map.csv'
np.savetxt(csv_filename, commands_new_with_throttling, delimiter=',', header=','.join(headers), comments='')




# 3D visualization

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

surf = ax.plot_surface(V, A, commands_new, cmap='viridis')

ax.set_xlabel('Velocity')
ax.set_zlabel('Acceleration')
ax.set_ylabel('Throttling Output')
ax.set_title('Neural Network Output vs. Velocity and Throttling')

fig.colorbar(surf)

plt.show()

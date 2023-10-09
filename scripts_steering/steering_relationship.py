import os
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
from sklearn.metrics import mean_absolute_error
from sklearn.metrics import r2_score
import math


data = pd.read_csv('steering_relationships.csv')
dataa = pd.read_csv('steering_relationships.csv')

# standardize data
mean0 = data['Steering_signal'].mean()
std0 = data['Steering_signal'].std()
data['Steering_signal'] = (data['Steering_signal'] - mean0) / std0

mean1 = data['Mean'].mean()
std1 = data['Mean'].std()
data['Mean'] = (data['Mean'] - mean1) / std1

X = data['Steering_signal'].values
y = data['Mean'].values

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.15, random_state=42)

# Convert NumPy arrays to PyTorch tensors
X_train = torch.tensor(X_train, dtype=torch.float32).unsqueeze(1)  # Add an extra dimension for the input feature
y_train = torch.tensor(y_train, dtype=torch.float32)
X_test = torch.tensor(X_test, dtype=torch.float32).unsqueeze(1)  # Add an extra dimension for the input feature
y_test = torch.tensor(y_test, dtype=torch.float32)



# NN model

class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.fc1 = nn.Linear(1, 128)
        self.relu1 = nn.ReLU()
        self.fc2 = nn.Linear(128, 512)
        self.relu2 = nn.ReLU()
        self.fc3 = nn.Linear(512, 1024)
        self.relu3 = nn.ReLU()
        self.fc4 = nn.Linear(1024, 256)
        self.relu4 = nn.ReLU()
        self.fc5 = nn.Linear(256, 1)


    def forward(self, x):
        x = self.fc1(x)
        x = self.relu1(x)
        x = self.fc2(x)
        x = self.relu2(x)
        x = self.fc3(x)
        x = self.relu3(x)
        x = self.fc4(x)
        x = self.relu4(x)
        x = self.fc5(x)


        return x
    


model = NeuralNetwork()

criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)


# training

num_epochs = 100
for epoch in range(num_epochs):
    outputs = model(X_train)

    loss = criterion(outputs, y_train.view(-1, 1))

    optimizer.zero_grad()
    loss.backward()
    optimizer.step()



with torch.no_grad():
    test_outputs = model(X_test)
    test_loss = criterion(test_outputs, y_test.view(-1, 1))


# predictions on new data
new_data = np.array([[(20-mean0)/std0], [(448-mean0)/std0]])
new_data = torch.tensor(new_data, dtype=torch.float32)
with torch.no_grad():
    predictions = model(new_data)*std1+mean1
    print('Predicted commands for new data:')
    for i, pred in enumerate(predictions):
        print(f"Data {i + 1}: {pred.item()}")




# evaluation
mse = mean_squared_error(y_test, test_outputs.view(-1).numpy())
print(f"Mean Squared Error on Test Data: {mse}")

mae = mean_absolute_error(y_test, test_outputs.view(-1).numpy())
print(f"Mean Absolute Error on Test Data: {mae}")

rmse = math.sqrt(mse)
print(f"Root Mean Squared Error on Test Data: {rmse}")

r2 = r2_score(y_test, test_outputs.view(-1).numpy())
print(f"R-squared (R2) Score on Test Data: {r2}")

test_outputs = test_outputs*std1+mean1
y_test = y_test*std1+mean1
X_test = X_test*std0+mean0


xdata = dataa.Steering_signal
ydata = dataa.Outer_wheel
zdata = dataa.Inner_wheel
kdata = dataa.Mean

#plt.figure(figsize=(10, 5))
#plt.scatter(X_test, y_test, label='True Data', color='b')
#plt.plot(X_test, test_outputs, label='Predictions', color='r', linewidth=2)
#plt.xlabel('Steering_signal')
#plt.ylabel('Mean')
#plt.legend()
#plt.show()

plt.scatter(xdata, zdata)
plt.scatter(xdata, ydata)
plt.scatter(xdata, kdata)
plt.show()
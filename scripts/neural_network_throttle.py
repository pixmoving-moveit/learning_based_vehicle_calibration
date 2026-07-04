#! /usr/bin/python3
import os
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
from sklearn.metrics import mean_absolute_error
from sklearn.metrics import r2_score
import math
import rclpy
from rclpy.node import Node

from map_quality_utils import (
    denormalize,
    normalize_columns,
    reject_outliers,
    report_coverage,
    save_checked_map,
    select_target_column,
)



class NeuralNetworkThrottle(Node):
    class NeuralNetwork(nn.Module):
        def __init__(self):
            super(NeuralNetworkThrottle.NeuralNetwork, self).__init__()
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

    def __init__(self):

        super().__init__('neural_network_throttle')

        self.model = self.NeuralNetwork()

        data = pd.read_csv('throttling.csv')
        ush = pd.read_csv('throttling.csv')
        target_column = select_target_column(
            data, "Acceleration_with_pitch_comp", "Acceleration_measured", self.get_logger()
        )


        # Declare params from launch file
        self.declare_parameter('filter_vel_throttle', 10.0)
        self.declare_parameter('filter_cmd_throttle', 10.0)
        self.declare_parameter('filter_acc_throttle', 10.0)

        # Load params from launch file
        self.FILTER_VEL_THROTTLE = self.get_parameter('filter_vel_throttle').get_parameter_value().double_value
        self.FILTER_CMD_THROTTLE = self.get_parameter('filter_cmd_throttle').get_parameter_value().double_value
        self.FILTER_ACC_THROTTLE = self.get_parameter('filter_acc_throttle').get_parameter_value().double_value

        required_columns = ["Velocity", "Throttling", target_column]
        data = data.replace([np.inf, -np.inf], np.nan).dropna(subset=required_columns)
        data = data[
            (data["Velocity"] >= 0.0)
            & (data["Throttling"] >= 0.0)
            & (data["Throttling"] <= 100.0)
            & (data[target_column] >= -8.0)
            & (data[target_column] <= 8.0)
        ]
        data = reject_outliers(
            data,
            {
                "Velocity": self.FILTER_VEL_THROTTLE,
                "Throttling": self.FILTER_CMD_THROTTLE,
                target_column: self.FILTER_ACC_THROTTLE,
            },
            self.get_logger(),
        )
        if len(data) < 10:
            raise RuntimeError("Not enough valid throttle samples after filtering.")

        dataa = data.copy()
        data, stats = normalize_columns(data, required_columns)


        # Split the data into input features (velocity and throttle) and target (acceleration) and test/train

        X = data[['Velocity', 'Throttling']].values
        y = data[target_column].values


        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)



        # Convert NumPy arrays to PyTorch tensors
        X_train = torch.tensor(X_train, dtype=torch.float32)
        y_train = torch.tensor(y_train, dtype=torch.float32)
        X_test = torch.tensor(X_test, dtype=torch.float32)
        y_test = torch.tensor(y_test, dtype=torch.float32)



        criterion = nn.MSELoss()
        optimizer = optim.Adam(self.model.parameters(), lr=0.001) #, weight_decay=0.001)


        # Training loop
        num_epochs = 100
        for epoch in range(num_epochs):
            # Forward pass
            outputs = self.model(X_train)
            
            loss = criterion(outputs, y_train.view(-1, 1))  

            # Backpropagation and optimization
            optimizer.zero_grad()  
            loss.backward()  
            optimizer.step() 


        with torch.no_grad():
            test_outputs = self.model(X_test)
            test_loss = criterion(test_outputs, y_test.view(-1, 1))
            #print(f"Mean Squared Error on Test Data: {test_loss.item()}")


        # Visualization

        velocity_range = np.linspace(0, dataa["Velocity"].max(), 20)
        throttling_range = np.linspace(0, dataa["Throttling"].max(), 20)
        report_coverage(
            dataa,
            "Velocity",
            "Throttling",
            target_column,
            velocity_range,
            throttling_range,
            self.get_logger(),
        )
        V, A = np.meshgrid(velocity_range, throttling_range)

        input_grid = np.column_stack(
            (
                (V.flatten() - stats["Velocity"][0]) / stats["Velocity"][1],
                (A.flatten() - stats["Throttling"][0]) / stats["Throttling"][1],
            )
        )
        input_grid = torch.tensor(input_grid, dtype=torch.float32)

        with torch.no_grad():
            commands = self.model(input_grid).reshape(V.shape)
            
            
        commands_new = denormalize(commands.numpy(), stats, target_column)



        # Save the trained model
        #torch.save(self.model.state_dict(), 'trained_throttle.pth')


        # evaluation
        y_test_real = denormalize(y_test.numpy(), stats, target_column)
        test_outputs_real = denormalize(test_outputs.view(-1).numpy(), stats, target_column)
        mse = mean_squared_error(y_test_real, test_outputs_real)
        self.get_logger().info(f"Mean Squared Error on Test Data: {mse}")

        mae = mean_absolute_error(y_test_real, test_outputs_real)
        self.get_logger().info(f"Mean Absolute Error on Test Data: {mae}")

        rmse = math.sqrt(mse)
        self.get_logger().info(f"Root Mean Squared Error on Test Data: {rmse}")

        r2 = r2_score(y_test_real, test_outputs_real)
        self.get_logger().info(f"R-squared (R2) Score on Test Data: {r2}")            


        # Save NN model in csv correct format for testing in the real vehicle

        csv_filename = 'accel_map.csv'
        commands_new = save_checked_map(
            csv_filename,
            velocity_range,
            throttling_range / 100.0,
            commands_new,
            True,
            "accel_map",
            self.get_logger(),
        )



        # 3D Visualization (plot)
        xdata = dataa.Velocity
        ydata = dataa.Throttling
        zdata = dataa[target_column]



        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        scatter = ax.scatter3D(xdata, ydata, zdata, c=zdata, marker='o')
        surf = ax.plot_surface(V, A, commands_new, cmap='viridis')

        ax.set_xlabel('Velocity')
        ax.set_zlabel('Acceleration')
        ax.set_ylabel('Throttling Output')
        ax.set_title('Neural Network Output vs. Velocity and Throttling')

        plt.figure(figsize=(10, 6))
        plt.subplot(3, 1, 1)
        plt.hist(ush['Velocity'], bins=20, color='skyblue', edgecolor='black')
        plt.title('Distribution of Velocity')
        plt.xlabel('Velocity')
        plt.ylabel('Frequency')

        # Plot the distribution of 'Throttling'
        plt.subplot(3, 1, 2)
        plt.hist(ush['Throttling'], bins=20, color='salmon', edgecolor='black')
        plt.title('Distribution of Throttling')
        plt.xlabel('Throttling')
        plt.ylabel('Frequency')

        # Plot the distribution of 'Acceleration_measured'
        plt.subplot(3, 1, 3)
        plt.hist(ush[target_column], bins=20, color='lightgreen', edgecolor='black')
        plt.title('Distribution of Acceleration')
        plt.xlabel('Acceleration')
        plt.ylabel('Frequency')

        plt.tight_layout()
    
        fig.colorbar(surf)

        plt.show()



def main():
    rclpy.init()
    neural_network_throttle = NeuralNetworkThrottle()
    rclpy.spin(neural_network_throttle)

    

if __name__ == '__main__':
    main()


import pandas as pd
import numpy as np
#from scipy.signal import medfilt

# Load the CSV file
df = pd.read_csv('throttling2.csv')

# Define columns of interest
columns = ["Velocity", "Throttling", "Acceleration_measured"]

# Step 1: Apply a median filter with a window size of 21 to each column
#window_size = 21
#for col in columns:
    #df[col] = medfilt(df[col], kernel_size=window_size)

# Step 2: Standardize the data for each column
for col in columns:
    mean = df[col].mean()
    std = df[col].std()
    df[col] = (df[col] - mean) / std

# Step 3: Remove outliers greater than 1 standard deviation from the mean
for col in columns:
    mean = df[col].mean()
    std = df[col].std()
    threshold = 1  # You can adjust this threshold as needed
    df = df[abs(df[col] - mean) <= threshold * std]

# Now, df contains the cleaned and standardized data

df.to_csv('throttling2_without_outliers.csv', index = False)


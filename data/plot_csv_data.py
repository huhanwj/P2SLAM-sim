import pandas as pd
import matplotlib.pyplot as plt

# Function to load data from a CSV file
def load_data_from_csv(file_path, header=None):
    return pd.read_csv(file_path, header=header)

# Define the paths to your CSV files
csv_file_1 = "DS1_noise.csv"
csv_file_2 = "DS1_gt.csv"
csv_file_3 = "optimized_poses.csv"

# Load data from the CSV files (with no header)
data1 = load_data_from_csv(csv_file_1)
data2 = load_data_from_csv(csv_file_2)
data3 = load_data_from_csv(csv_file_3)

# Plot data using the specified columns for x and y
plt.plot(data1.iloc[:, 0], data1.iloc[:, 1], label='noisy')
plt.plot(data2.iloc[:, 0], data2.iloc[:, 1], label='ground truth')
plt.plot(data3.iloc[:, 1], data3.iloc[:, 2], label='optimize only using gt')

# Customize the plot
plt.xlabel('X-axis label')
plt.ylabel('Y-axis label')
plt.title('Plot Title')
plt.legend()

# Save the plot to a file (optional)
plt.savefig("output_plot.png")

# Show the plot
plt.show()

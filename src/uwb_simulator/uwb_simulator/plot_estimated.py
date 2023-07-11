import numpy as np
import pandas as pd
import csv
import seaborn
import matplotlib.pyplot as plt

# Read the csv file with the estimated positions
estimated_data = pd.read_csv('data.csv')
print(estimated_data.head())
# Plot the estimated positions vs the real positions only for the x and y axis
seaborn.scatterplot(data=estimated_data, x='x', y='y', color='red')
seaborn.scatterplot(data=estimated_data, x='estimated_x', y='estimated_y', hue='label')
plt.show()


import numpy as np
import pandas as pd
import csv
import dask.dataframe as dd

from matplotlib import pyplot as plt
from dask.diagnostics import ProgressBar

# Read the csv of the measurements
data_csv = pd.read_csv('./measurements_std_0.0.csv')

# print(data_csv.head())

col_names = data_csv.columns
# print (col_names)
# Match the columns with the ones that start with from_
ranges = [col for col in col_names if col.startswith('from_')]
# print(ranges)
# Match the columns with the ones that start with GT_
anchors = [col for col in col_names if col.startswith('GT_')]
# print(ground_truth)
# Match the rest of the columns that are the uwb antennas not in the ground truth and not the ranges
uwb_antennas = [col for col in col_names if col not in ranges and col not in anchors]
# print(uwb_antennas)
# Remove the las two characters of the names of the antennas and apply unique to get the names of the robots
robots = np.unique([antenna[:-2] for antenna in uwb_antennas])
print(robots)

# uwb_dist = data_csv[ranges].to_numpy()
# print(uwb_dist)
# platform_pose = data_csv[anchors].to_numpy()
# print(platform_pose)
# opti_pose = data_csv[uwb_antennas].to_numpy()
# print(opti_pose)

# Declare array of estimated positions
final_data = pd.DataFrame(columns=['antenna', 'x', 'y', 'z', 'range'])
final_data = []

# Create the variables to save the data in a csv file
with open(f'data.csv', 'w') as f:
    writer = csv.writer(f)
    cols = ['label', 'x', 'y', 'z', 'estimated_x', 'estimated_y', 'estimated_z', 'error']
    writer.writerow(cols)

# ranges = uwb_dist[180]
# ground_truth = platform_pose[180]
# uwb_antennas = opti_pose[180]

print('Ranges:')
print(ranges)
print('Anchors:')
print(anchors)
print('UWB antennas:')
print(uwb_antennas)
print('robots:')
print(robots)
print()

# # Create matrix of the values to iterate over
# for robot in robots:
#     # Filter the columns of the original dataframe that either match the robots name or the ground truth
#     filtered_cols = data_csv.filter(regex=f'{robot}|GT_')
#     print('Filtered columns:')
#     print(f'{filtered_cols.head()}\n')
#     # Gropby the columns of the anchors to have the coordinates of the ground truth in a single row per index
#     grouped = filtered_cols.filter(regex=f'GT_').groupby(np.arange(len(anchors)) // 3, axis=1).groups
#     # Print the grouped columns
#     print('Grouped columns:')
#     print(grouped)

#     for key, cols in grouped.items():
#         print(f'key = {key} cols = {cols}')
#         # Get the values of the columns
#         anchor_coordinates = filtered_cols[cols].values
#         print(f'anchor_coordinates = {anchor_coordinates}')
#     # Iterate over each of the groups
#     break

def relative_position_from_four_meas(offsets, measurements, limitx = [-1,8], limity = [-1,8], limitz = [0 , 1.5]) :
    '''
        Simple least squares approach for (up to) 4 anchors on the UGv

        Measurements is a list of length num_of_anchors
    '''

    min_err = None
    min_err_pos = [0, 0]

    xys = np.mgrid[limitx[0]:limitx[1]:0.01, limity[0]:limity[1]:0.01, limitz[0]:limitz[1]:0.1].reshape(3, -1).T
    # print(xys)

    err = None
    for idx, m in enumerate(measurements) :
        if err is None :
            err = ( m - np.linalg.norm(xys - offsets[idx], axis=1) ) ** 2
        else :
            err += ( m - np.linalg.norm(xys - offsets[idx], axis=1) ) ** 2
    # err = ( measurements - numpy.linalg.norm(xys - offsets[idx], axis=1) ) ** 2
    
    print(f'error shape = {err.shape}')
    # Plot the heatmap error of the position in (x, y)
    # fig, ax = plt.subplots()
    # im = ax.imshow(err.reshape(81, 81), cmap='hot', interpolation='nearest', extent=[limitx[0], limitx[1], limity[0], limity[1]])
    # fig.colorbar(im, ax=ax)
    # plt.show()


    min_err_idx = np.argmin(err)
    min_err = err[min_err_idx]
    min_err_pos = xys[min_err_idx]
    # print(min_err_pos, min_err)
    return min_err_pos, min_err

# Map function to process each row of the dataframe
def process_row(row):
    # print(f'Estimating position... ', end='')
    # print(row)
    # Get the ranges
    ranges_values = row[ranges].values
    # print(f'\nranges_ = {ranges_values}')
    # Get the ground truth
    ground_truth = row[anchors].values
    # print(f'ground_truth_ = {ground_truth}')
    # Get the uwb antennas
    uwb_opti_positions_ = row[uwb_antennas].values
    # print(f'uwb_opti_positions_ = {uwb_opti_positions_}\n')
    # Reshape the ground truth to a 3xn matrix
    ground_truth = ground_truth.reshape((3, -1))
    # print(f'ground_truth = {ground_truth}')

    # Iterate over each of the uwb antennas
    for i in range(0, len(uwb_antennas), 3):
        # print(i)
        # Get the x, y, z of the antenna
        x, y, z = uwb_opti_positions_[i:i+3]
        # Get the name of the current antenna
        uwb_name = uwb_antennas[i][0:-2]
        # Filter the ranges that have the name of the antenna
        distance_indexes = [i for i, col in enumerate(ranges) if uwb_name in col]
        distances = ranges_values[distance_indexes]
        # Get the corresponding range column name
        range_name = [col for col in ranges if uwb_name in col]
        # Print the results
        # print(f'Antenna: {uwb_antennas[i]} robot: {uwb_name}, x: {x}, y: {y}, z: {z}')
        print(f'ground_truth_indexes = {range_name}')
        print(f'ground_truth = {ground_truth}')
        print(f'distance_indexes = {distance_indexes}, distances = {distances}')

        # Get the estimated position and the error
        estimated_pose, error = relative_position_from_four_meas(ground_truth, distances)
        # Save the estimated position, the real position and the error
        data = [uwb_name, x, y, z, estimated_pose[0], estimated_pose[1], estimated_pose[2], error]
        final_data.append(data)
        # Save the estimated positions to a csv
        with open(f'data.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow(data)
        print(f'x: {x}, y: {y}, z: {z}, estimated_x: {estimated_pose[0]}, estimated_y: {estimated_pose[1]}, estimated_z: {estimated_pose[2]}, error: {error} \n')
    return

# # Generate dask dataframe
# ddf = dd.from_pandas(data_csv, npartitions=3)
# ddf_updated = ddf.apply(process_row, axis=1, meta=('x', 'f8'))

# # Save dataframes to csv
# ddf_updated.to_csv('data.csv', single_file=True)

# Apply the function to each row of the dataframe
data_csv.apply(process_row, axis=1)

# Save the data to a csv
# with open(f'data.csv', 'w') as f:
#     writer = csv.writer(f)
#     writer.writerow(['uwb_name', 'x', 'y', 'z', 'estimated_x', 'estimated_y', 'estimated_z', 'error'])
#     writer.writerows(final_data)

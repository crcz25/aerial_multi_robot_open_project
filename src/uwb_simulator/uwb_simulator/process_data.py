import numpy as np
import pandas as pd
import csv
import dask.dataframe as dd
import ast
import re
import argparse

from _localization import lse

def process_row(row, uwb_antenna, ranges, anchors):
    # print(f'Estimating position... ')
    # print(row)
    # Get the antennas coordinates of the row
    antenna_pos = row[uwb_antenna]
    # print(antenna_pos)
    # Filter the ranges that correspond to the antenna
    reg = re.compile(f'.*to_{uwb_antenna}.*')
    cols = list(filter(reg.match, ranges))
    # print(cols)
    # Get the ranges of the antenna
    antenna_ranges = row[cols].to_numpy()
    # Construct the matrix of the anchor positions (ground truth)
    anchor_pos = row[anchors].apply(ast.literal_eval).to_numpy()
    # print(anchor_pos)
    # Calculate the relative position of the antenna
    pos, err = lse(anchor_pos, antenna_ranges)
    # print(f'Real position: {antenna_pos}')
    # print(f'Estimated pos {pos}, err {err}')
    # print()

    return (pos.tolist(), err)

def main():
    # Get the name of the file from the parameters
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, help='Name of the file to process')
    args = parser.parse_args()
    name = args.file

    # Read the csv of the measurements
    data_csv = pd.read_csv(name)
    # Remove all the rows that have a nan value
    data_csv = data_csv.dropna()
    # Reset the index of the dataframe
    data_csv = data_csv.reset_index(drop=True)
    print(data_csv)
    # Get the columns of the csv
    col_names = data_csv.columns
    print (col_names)
    # Match the columns with the ones that start with from_
    ranges = [col for col in col_names if col.startswith('from_')]
    print(ranges)
    # Match the columns with the ones that start with GT_
    anchors = [col for col in col_names if col.startswith('GT_')]
    # Convert string of positions to list of floats
    print(anchors)
    # Match the rest of the columns that are the uwb antennas not in the ground truth and not the ranges
    uwb_antennas = [col for col in col_names if col not in ranges and col not in anchors]
    # Convert string of positions to list of floats
    print(uwb_antennas)
    # Remove the las two characters of the names of the antennas and apply unique to get the names of the robots
    robots = np.unique([antenna[:-2] for antenna in uwb_antennas])
    print(robots)
    # Define the structure of the output file
    output = pd.DataFrame(columns=uwb_antennas)
    print(1+data_csv.memory_usage(deep=True).sum()//10)
    # ddf_out = dd.from_pandas(data_csv, npartitions=1).repartition(partition_size=f'{1+data_csv.memory_usage(deep=True).sum() // 10}MB')
    ddf_out = dd.from_pandas(data_csv, npartitions=10)
    print()

    # Iterate over the antennas to estimate the position
    for uwb_antenna in uwb_antennas:
        print(f'Estimating position for antenna {uwb_antenna}...')
        # Calculate the relative position of the antenna
        # output[uwb_antenna] = data_csv.apply(process_row, args=(uwb_antenna, ranges, anchors, output, ), axis=1)
        ddf_res = ddf_out.apply(process_row, args=(uwb_antenna, ranges, anchors, ), axis=1, meta=pd.Series(dtype='object'))
        # print(ddf_res)
        # Save the estimated position into the output dataframe
        output[uwb_antenna] = ddf_res
    # Add the positions of the anchors to the output dataframe
    for anchor in anchors:
        output[anchor] = data_csv[anchor]
    # Save the output dataframe to a csv file
    output.to_csv('output.csv', index=False)

    pass

if __name__ == "__main__":
    main()

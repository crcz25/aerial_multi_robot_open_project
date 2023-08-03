import numpy as np
import pandas as pd
import re
import seaborn as sns
import ast
import matplotlib.pyplot as plt
import argparse

def main():
    # Get the name of the file from the parameters
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, help='Name of the file to process', default='output.csv')
    args = parser.parse_args()
    name = args.file

    # Read the csv file with the estimated positions
    estimated_data = pd.read_csv(name)
    print(estimated_data.head())
    # Filter the ranges that correspond to the antenna
    reg = re.compile(f'GT_.*')
    gt_cols = list(filter(reg.match, estimated_data.columns))
    print(gt_cols)
    # Get the rest of the antenna columns
    rest_cols = [col for col in estimated_data.columns if col not in gt_cols]
    print(rest_cols)


    # sns.scatterplot(x='x', y='y', data=estimated_data, hue='GT_1')

    fig, ax = plt.subplots()
    # Plot the estimated positions vs the real positions only for the x and y axis
    for antenna in rest_cols:
        pos, err = zip(*estimated_data[antenna].apply(ast.literal_eval))
        pos = np.array(pos)
        # print(pos)
        # Plot the real position
        sns.scatterplot(x=pos[:, 0], y=pos[:, 1], ax=ax, label=antenna)

    for antenna in gt_cols:
        pos = estimated_data[antenna].apply(ast.literal_eval).to_numpy()
        pos_x = [p[0] for p in pos]
        pox_y = [p[1] for p in pos]
        # Plot the real position
        sns.scatterplot(x=pos_x, y=pox_y, ax=ax, label=antenna, marker='o', color='black')
        # Add text of the point to the plot
        for i, txt in enumerate(antenna):
            ax.annotate(txt, (pos_x[i], pox_y[i]))
            
        # break
    plt.show()

    return

if __name__ == "__main__":
    main()


#! /usr/bin/python
import argparse
import ast
import csv
import sys
from itertools import combinations, permutations, product
from pathlib import Path
from random import randint
from time import time

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rclpy
import seaborn as sns
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from tf_transformations import euler_from_quaternion

from tello_msgs.srv import TelloAction


def main(args=None):
    csv_file = (
        Path.cwd()
        / "src"
        / "uwb_real_trajectories"
        / f"positions1696328164.6783803.csv"
    )

    # Read the csv file with the estimated positions
    estimated_data = pd.read_csv(csv_file)
    print(estimated_data.head())

    # Transform each cell in the row to x,y,z coordinates
    cols = estimated_data.columns
    for col in cols:
        pos = estimated_data[col].apply(ast.literal_eval)
        pos_x = [p[0] for p in pos]
        pos_y = [p[1] for p in pos]
        pos_z = [p[2] for p in pos]
        # Plot the positions
        sns.scatterplot(x=pos_x, y=pos_y, label=col)
    plt.show()



if __name__ == "__main__":
    main()

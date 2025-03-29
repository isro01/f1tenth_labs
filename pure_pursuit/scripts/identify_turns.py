import pandas as pd
import numpy as np

csv_input_file = '/home/vulcan/f1tenth/labs/waypoints/logs/anotherloop.csv'
df = pd.read_csv(csv_input_file, header=None), names=['x', 'y', 'orientation', 'speed']


import csv
# import atexit
from scipy.interpolate import splprep, splev
import numpy as np

class Smoother():

    def __init__(self):
        # Load waypoints from csv file
        self.trajectory = []
        self.new_trajectory = []
        self.headings = []

        try:
            with open('/home/vulcan/f1tenth/labs/waypoints/logs/hunanfinal.csv', 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    self.trajectory.append([float(row[0]), float(row[1])])
                    self.headings.append(float(row[2]))
                print("Loaded waypoints from CSV file")
        except Exception as e:
            # self.get_logger().error(f"Error loading waypoints: {e}")
            print(f"Error loading waypoints: {e}")


        self.smoothing_function()

        print("Generated smoother trajectory!")

        # Save back to csv file
        with open('/home/vulcan/f1tenth/labs/waypoints/logs/smooth_hunanfinal.csv', 'w') as f:
            writer = csv.writer(f)
            for row in self.new_trajectory:
                writer.writerow(row)

    def smoothing_function(self):

        # Downsample
        self.sampled_down_trajectory = self.trajectory[::20]
        # self.sampled_down_trajectory = self.sampled_down_trajectory[:-50]
        
        np_trajectory = np.array(self.sampled_down_trajectory)
        # print(np_trajectory.shape)
        tck, u = splprep(np_trajectory.T, s=0.1)
        u_new = np.linspace(u.min(), u.max(), 500)
        new_u = splev(u_new, tck)
        self.new_trajectory = np.array(new_u).T.tolist()

        
def main(args=None):
    obj = Smoother()


if __name__ == '__main__':
    main()
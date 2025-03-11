import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import csv
import atexit
from scipy.interpolate import splprep, splev
import numpy as np

class Smoother(Node):

    def __init__(self):

        super().__init__('smoother')

        # Load waypoints from csv file
        self.trajectory = []
        self.new_trajectory = []

        try:
            with open('/home/vulcan/f1tenth/labs/waypoints/logs/mimic_locker.csv', 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    self.trajectory.append([float(row[0]), float(row[1])])
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")


        self.smoothing_function()

        # Save back to csv file
        with open('/home/vulcan/f1tenth/labs/waypoints/logs/smoothed_mimic_locker.csv', 'w') as f:
            writer = csv.writer(f)
            for row in self.new_trajectory:
                writer.writerow(row)

        self.get_logger().info("Smoothing complete")
        self.shutdown()


    def smoothing_function(self):

        # Downsample
        self.sampled_down_trajectory = self.trajectory[::20]

        # Convert to numpy array
        np_trajectory = np.array(self.sampled_down_trajectory)
        print(np_trajectory.shape)
        tck, u = splprep(np_trajectory.T, s=0.1)
        u_new = np.linspace(u.min(), u.max(), 5000)
        new_u = splev(u_new, tck)
        self.new_trajectory = np.array(new_u).T.tolist()

    def shutdown(self):
        self.get_logger().info("Goodbye")



        
def main(args=None):
    rclpy.init(args=args)
    node = Smoother()
    atexit.register(node.shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
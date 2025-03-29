import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import csv
import atexit


class WaypointsViz(Node):
    def __init__(self):
        super().__init__('waypoints_viz')

        # Load waypoints from csv file
        self.trajectory = []

        try:
            with open('/home/vulcan/f1tenth/labs/waypoints/logs/manual.csv', 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    self.trajectory.append([float(row[0]), float(row[1])])
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
    
        # Create publisher for waypoints
        self.publisher = self.create_publisher(MarkerArray, 'all_waypoints', 10)
        self.timer = self.create_timer(0.1, self.publish_waypoints)

    
    def publish_waypoints(self):
        marker_array = MarkerArray()

        # print(len(self.trajectory))
        self.sampled_down_trajectory  = self.trajectory[::1]
        temp1 = self.sampled_down_trajectory[0][0]
        temp2 = self.sampled_down_trajectory[0][1]
        
        for i, point in enumerate(self.sampled_down_trajectory):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            # marker.pose.position = Point(x=point[1] - temp2 + 0.2, y=point[0] - temp1 - 3.1, z=0.0)
            # marker.pose.position = Point(x=point[0] - 6.1, y= -20-point[1], z=0.0)
            marker.pose.position = Point(x=point[0], y=point[1], z=0.0)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.3
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            marker_array.markers.append(marker)
        
        self.publisher.publish(marker_array)

    def shutdown(self):
        self.get_logger().info("Goodbye")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointsViz()
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


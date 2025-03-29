import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_pose_once():
    rclpy.init()
    node = rclpy.create_node('single_pose_publisher')
    
    publisher = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    
    msg.pose.covariance = [0.0] * 36  # 6x6 covariance matrix
    
    publisher.publish(msg)
    node.get_logger().info('Published PoseWithCovarianceStamped message')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    publish_pose_once()

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker


class ConeSlamSubscriber(Node):

    def __init__(self):
        super().__init__('cone_slam_sub')
        self.subscription = self.create_subscription(
            Marker,
            '/slam/cones_positions',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'i see {len(msg.points)} cones')


def main(args=None):
    rclpy.init(args=args)

    cone_slam_sub = ConeSlamSubscriber()

    rclpy.spin(cone_slam_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cone_slam_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

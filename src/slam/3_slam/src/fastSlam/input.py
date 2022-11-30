import rclpy
from rclpy.node import Node
from eufs_msgs.msg import WheelSpeedsStamped
from algorithm import *

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('slam_odometry')
        self.subscription = self.create_subscription(
            WheelSpeedsStamped,
            '/ros_can/wheel_speeds',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.speeds)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

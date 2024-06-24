import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Odometry, "odom", self.listener_callback, 10)

        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.pose.pose.position)


def main(args=None):
    rclpy.init(args=args)

    publisher_2 = MinimalPublisher()

    rclpy.spin(publisher_2)
    publisher_2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

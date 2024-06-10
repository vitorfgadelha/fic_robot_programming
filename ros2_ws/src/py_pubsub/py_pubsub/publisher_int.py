import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_2')
        self.publisher_ = self.create_publisher(Int64, 'topic_int', 10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int64()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher_2 = MinimalPublisher()

    rclpy.spin(publisher_2)
    publisher_2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

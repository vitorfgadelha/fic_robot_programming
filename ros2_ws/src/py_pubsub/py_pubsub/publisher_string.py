import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_1')
        self.publisher_ = self.create_publisher(String, 'topic_string', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'A mensagem é: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('"%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher_1 = MinimalPublisher()

    rclpy.spin(publisher_1)
    publisher_1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node

from interfaces.msg import Id

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher_2')
        self.publisher_ = self.create_publisher(Id, 'id_data', 10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Id()
        msg.first_name = "Vitor"
        msg.last_name = "Gadelha"
        msg.age = 27
        msg.height = 1.64
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.first_name)


def main(args=None):
    rclpy.init(args=args)

    publisher_2 = MinimalPublisher()

    rclpy.spin(publisher_2)
    publisher_2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

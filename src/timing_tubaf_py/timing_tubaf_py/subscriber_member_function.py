import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node): #class for the node

    def __init__(self): #constructor for the subsbriber
        super().__init__('py_subscriber') #names the subscriper "py_subscriber"
        self.publisher_ = self.create_publisher(
            String, 
            'diff',
            10)
        self.subscription = self.create_subscription(
            String, #message type
            'number', #topic name
            self.listener_callback,
            10)
        self.subscription
        self.publisher_

    def listener_callback(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        #prints info message to the console


def main(args=None):
    rclpy.init(args=args)

    py_subscriber = MinimalSubscriber()

    rclpy.spin(py_subscriber)

    py_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

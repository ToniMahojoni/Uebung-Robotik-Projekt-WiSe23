import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node): #class for the node

    def __init__(self): #constructor for the subsbriber
        super().__init__('py_subscriber') #names the subscriper "py_subscriber"
        self.subscription = self.create_subscription(
            int, #message type
            'number', #topic name
            self.listener_callback,
            10)
        self.subscription 

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data) 
        #prints info message to the console


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

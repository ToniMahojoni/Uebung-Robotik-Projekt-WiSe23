# ros2 framework for the node
import rclpy
from rclpy.node import Node

# message types for string and integer messages
from std_msgs.msg import String
from std_msgs.msg import Int32

# access to system time and time calculations
from datetime import datetime

# class for the node
class MinimalSubscriber(Node):
    
    #constructor for the subsbriber/publisher
    def __init__(self): 
        # name for the subscriber/publisher
        super().__init__('py_subpub')

        # publisher for string messages on diff topic
        self.publisher = self.create_publisher(
            String, 
            'diff',
            10)
        
        # subscriber for integer messages on number topic
        # with listener_callback function defined below
        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.listener_callback,
            10)
        
        # declaration of subscriber and publisher fields
        self.subscription
        self.publisher

        # declaration of time field for calculations
        self.time = datetime.now()

    # function creating a message with the time difference
    def listener_callback(self, msg):

        # prints confirmation message to console
        self.get_logger().info('I heard: "%i"' % msg.data)
        
        # time variable for saving the time when message arrived
        message_time = datetime.now()

        # changing the message to a string with the time difference
        msg = String()
        msg.data = 'Zeitdifferenz: ' + str(message_time - self.time)

        # publishing the message to the topic defined in the publisher
        self.publisher.publish(msg)
        
        # changing the time variable for the next calculation
        self.time = message_time

# main function
def main(args=None):

    # initializing rclpy library
    rclpy.init(args=args)

    # creation of the node
    py_subpub = MinimalSubscriber()

    # spinning the node so that callbacks get executed
    rclpy.spin(py_subpub)

    # destroying the node and shutting down the rclpy library
    py_subpub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

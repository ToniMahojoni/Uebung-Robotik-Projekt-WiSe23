# ros 2 framework for the node
import rclpy
import rclpy.node

# message types for the string, scan data and robot movement messages
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# numpy for array computations
import numpy as np

# class for the node
class DrivingLogic(rclpy.node.Node):

    # constructor
    def __init__(self):
        super().__init__('obstacle_chasing')

        # definition of the changeable parameters
        self.declare_parameter('near_stop_distance', 0.2)
        self.declare_parameter('far_stop_distance', 0.5)
        self.declare_parameter('speed_chasing', 0.1)
        self.declare_parameter('speed_turn', 0.2)
        self.declare_parameter('scan_angle', 10)

        # definition of the QoS for receiving data
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # creation of the subscriber for the scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        
        # prevent unused variable warning
        self.subscription  

        # creation of the publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # timer to periodically invoke driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

        # defining variables for saving laser data
        self.smallest_distance = 0.0
        self.smallest_index = 0

    # function for handling laser scan data
    def scanner_callback(self, msg):

        # variable with biggest possible distance for finding the smallest distance later
        shortest_distance = self.get_parameter('far_stop_distance').get_parameter_value().double_value
        shortest_index = 0

        # save current measurement data
        current_measure = np.array(msg.ranges)

        # iterates through all the laser beams and searches for the one with least distance
        for i in range(self.get_parameter('scan_angle').get_parameter_value().integer_value):
            if (current_measure[i] < shortest_distance) and (current_measure[i] > 0.0):
                shortest_distance = current_measure[i]
                shortest_index = i

        # shortest distance with its index gets saved
        self.last_distance = shortest_distance
        self.last_index = shortest_index

    def timer_callback(self):

        # create variable with parameter values
        near_stop_distance = self.get_parameter('near_stop_distance').get_parameter_value().double_value

        # driving logic

        # stop and turn if nearest obstacle isn't in front
        if (self.last_index > 0):
            speed = 0.0
            turn = self.get_parameter('speed_turn').get_parameter_value().double_value
            print('turn')
        # stop if nearest obstacle is too close
        elif (self.last_distance <= near_stop_distance):
            speed = 0.0
            turn = 0.0
            print('stop')
        # drive if all conditions are fulfilled
        else:
            speed = self.get_parameter('speed_chasing').get_parameter_value().double_value
            turn = 0.0
            print('drive')

        # create message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        #send message
        self.publisher_.publish(msg)

# main function
def main(args=None):

    # program start and initializing rclpy library
    print('Program start.')
    rclpy.init(args=args)

    # creation of the node
    node = DrivingLogic()

    # spinning the node so that callbacks get executed
    rclpy.spin(node)

    # destroying the node and shutting down the rclpy library
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
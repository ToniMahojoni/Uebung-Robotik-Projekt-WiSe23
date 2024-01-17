# ros 2 framework for the node
import rclpy
import rclpy.node

# message types for the string, scan data and robot movement messages
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

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
        self.declare_parameter('scan_angle', 0)

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
    self.publisher_ = self.create_publisher(Twist, 'velocity', 1)

    # timer to periodically invoke driving logic
    timer_period = 0.5  # seconds
    self.my_timer = self.create_timer(timer_period, self.timer_callback)

    # function for handling laser scan data
    def scanner_callback(self, msg):

        # sets start distance to first measurement
        least_distance = msg.ranges[0]

        # iterates through all the laser beams and searches for the one with least distance
        for i in range(self.get_parameter('scan_angle').get_parameter_value().integer_value):
            if msg.ranges[i] < least_distance:
                least_distance = msg.ranges[i]
                least_index = i

        # laser beam value and index with least distance gets saved and printed 
        self.last_distance = least_distance
        self.last_index = least_index
        print(self.last_distance)
        print(self.last_index)

    def timer_callback(self):

        # create variables with parameter values
        near_stop_distance = self.get_parameter('near_stop_distance').get_parameter_value().double_value
        far_stop_distance = self.get_parameter('far_stop_distance').get_parameter_value().double_value

        # no far away or too close obstacles
        if (self.last_distance > far_stop_distance) or (self.last_distance < near_stop_distance):
            speed = 0.0
            print('stop')

        # turn if nearest obstacle isn't in front
        if (self.last_index > 0):
            turn = self.get_parameter('speed_turn').get_parameter_value().double_value
            print('turn')
        else:
            turn = 0.0

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
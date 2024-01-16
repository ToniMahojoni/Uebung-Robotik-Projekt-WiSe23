// standard c++ headers for basic functionalities
#include <chrono>
#include <functional>
#include <memory>

// ros2 framework for creating the node
#include "rclcpp/rclcpp.hpp" 

// message type for int32 messages for the publisher
#include "std_msgs/msg/int32.hpp" 

using namespace std::chrono_literals;

// class for the node
class MinimalPublisher : public rclcpp::Node 
{
public:

  // constructor for the node with name "cpp_publisher" and counter set to 0
  MinimalPublisher()
  : Node("cpp_publisher"), count_(0) 
  {

    // initializing publisher with message type Int32, topic number and queue size 10
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("number", 10);

    // initializing timer for executing the timer_callback function every second
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:

  // function for setting message data and publishing the message
  void timer_callback()
  {

    // message data set to incrementing integer counter
    auto msg = std_msgs::msg::Int32();
    msg.data = count_++;

    // printing a confirmation message to the console
    RCLCPP_INFO(this->get_logger(), "Integer Value: '%i'", msg.data);
    
    // publishing the message to the defined topic
    publisher_->publish(msg);
  }

  //declaration of timer, publisher and counter fields
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  int32_t count_;
};

// main function
int main(int argc, char * argv[])
{

  //initializing rclcpp library
  rclcpp::init(argc, argv);

  // spinning the node so that callbacks get executed
  rclcpp::spin(std::make_shared<MinimalPublisher>()); 

  // shutting down rclcpp library
  rclcpp::shutdown();
  return 0;
}

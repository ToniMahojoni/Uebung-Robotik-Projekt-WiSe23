#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp" // ros2 features
#include "std_msgs/msg/string.hpp" // includes message-type for publishing

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node //class for the node
{
public:
  MinimalPublisher()
  : Node("cpp_publisher"), count_(0) 
  //constructor creating the node with name "cpp_publisher" and sets counter to 0
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    /* initializing the publisher with "int"-message type, topic "number" and the
    required queue size to limit messages in backup */ 
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    // initializing timer causing exectution of timer_callback funciton every second 
  }

private:
  void timer_callback() //sets message data and publishes them
  {
    auto msg = std_msgs::msg::String();
    msg.data = std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    // ensures that every published message is printed to console
    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  //declaration of timer, publisher and counter fields
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //initializing ROS2
  rclcpp::spin(std::make_shared<MinimalPublisher>()); 
  //processes data from the node (callbacks from the timer)
  rclcpp::shutdown();
  return 0;
}

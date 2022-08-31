// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions

// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
 
// Built-in message type that will be used to publish data
#include "std_msgs/msg/float64.hpp"

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;
 
// Create the node class named MinimalPublisher which inherits the attributes
// and methods of the rclcpp::Node class.
class MinimalPublisher : public rclcpp::Node
{
  public:
    // Constructor creates a node named minimal_publisher. 
    // The published message count is initialized to 0.
    MinimalPublisher()
    : Node("Target_attitude"), count_(0)
    {
      // Publisher publishes String messages to a topic named "addison". 
      // The size of the queue is 10 messages.
      publisher_ = this->create_publisher<std_msgs::msg::Float64>("theta_tgt", 0);

      // publish at 1 milliseconds intervals
      timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this)); // 遅いとき: 10 ms
    }
 
  private:
    // This method executes every 500 milliseconds
    void timer_callback()
    {
      auto message = std_msgs::msg::Float64();

      // 1 ms 毎にcountがインクリメントされていき、10 s で360増える
      message.data = double(count_++) * 360/1000 * 1/10; // 通常: * 1/1000 遅いとき: * 1/100
      //message.data = double(count_++) * 360/10 * 1/100; // 通常: * 1/1000 遅いとき: * 1/100

      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
       
      // Publish
      publisher_->publish(message);

    }
     
    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;
  
    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
   
    // Declaration of the count_ attribute
    size_t count_;
};
 
// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<MinimalPublisher>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}

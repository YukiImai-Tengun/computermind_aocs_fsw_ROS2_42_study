// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <math.h>
#include <fstream>
#include <sys/time.h>

#include <iostream> //標準入出力
#include <sys/socket.h> //アドレスドメイン
#include <sys/types.h> //ソケットタイプ
#include <arpa/inet.h> //バイトオーダの変換に利用
#include <unistd.h> //close()に利用



// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"


// Built-in message type that will be used to publish data
#include "std_msgs/msg/float64.hpp"


// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;
 
// Create the node class named MinimalPublisher which inherits the attributes
// and methods of the rclcpp::Node class.
class MinimalPublisher : public rclcpp::Node
{
  public:

  rclcpp::Clock::SharedPtr clock_;


    // Constructor creates a node named minimal_publisher. 
    // The published message count is initialized to 0.
    MinimalPublisher()
    : Node("Test_command"), count_(0)
    {
      publisher_6 = this->create_publisher<std_msgs::msg::Float64>("Torque", 10);

      timer_5 = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::timer_callback_5, this));
    }
 
  private:

  double value = 0;
  std::string seq;


    void timer_callback_5()
    {
      // Create a new message of type 
      auto message = std_msgs::msg::Float64();
      

      message.data = 1;

      seq = "1";
      RCLCPP_INFO(this->get_logger(), seq);

      




      //publisher_6->publish(message);
      
    }

    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_5;

    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_6;  

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

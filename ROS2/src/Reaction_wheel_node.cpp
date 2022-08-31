// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <fstream>

// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
 
// Built-in message type that will be used to publish data
#include "std_msgs/msg/float64.hpp"
 
// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;
using std::placeholders::_1;
 


// Create the node class named MinimalPublisher which inherits the attributes
// and methods of the rclcpp::Node class.
class MinimalPubSub : public rclcpp::Node
{
  public:
    // Constructor
    // The name of the node is minimal_subscriber
    MinimalPubSub()
    : Node("Reaction_wheel")
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "T_rw_command", 100, std::bind(&MinimalPubSub::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<std_msgs::msg::Float64>("Torque", 100);
    }
 

  private:
    // Receives the String message that is published over the topic
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
      // Write the message that was received on the console window

      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);

      ///////////////////////////////////////////////////////////////////////////////////

      // 時間取得部分
      //rclcpp::TimeSource ts(shared_from_this());
      //rclcpp::Clock::SharedPtr clk2 = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      //ts.attachClock(clk2);
      //rclcpp::Clock clk3;

      //RCLCPP_INFO(this->get_logger(), "clk2 time: %ld", clk2->now().nanoseconds()); // clk2はROS時間を使用し、clk3はデフォルトでシステム時間 しかし、sim時間をオンにしない限り、それらは常に同じになります
      //RCLCPP_INFO(this->get_logger(), "clk3 time: %ld", clk3.now().nanoseconds()); // 
      //RCLCPP_INFO(this->get_logger(), "this->now time: %ld", this->now().nanoseconds()); // 
      //RCLCPP_INFO(this->get_logger(), "CONTACT!!! time: %ld", std::chrono::system_clock::now()); // システム時間
      

      // ファイル書き込み部分
      #if 0      
      std::ofstream writing_file;
      std::string filename = "Reaction_Wheel_T_rw_command_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(msg->data) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////


      publisher_->publish(*msg);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg->data);

    }


    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};
 


// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<MinimalPubSub>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();

  return 0;
}

   







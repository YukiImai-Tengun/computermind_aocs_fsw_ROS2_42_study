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
    : Node("Gryo_sensor")
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      subscription_1_ = this->create_subscription<std_msgs::msg::Float64>(
      "omega_raw", 10, std::bind(&MinimalPubSub::topic_callback_1, this, _1));
      publisher_1_ = this->create_publisher<std_msgs::msg::Float64>("omega_Gyro", 1);

      update_timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPubSub::update_callback, this));

    }
 

  private:
    double omega_ = 0.0;

    void topic_callback_1(const std_msgs::msg::Float64::SharedPtr msg)
    {

      //theta_ST.data = msg->data;
      omega_ = msg->data;

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
      std::string filename = "Gyro_sensor_omega_raw_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(omega_) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////


      auto omega_Gyro = std_msgs::msg::Float64();
      omega_Gyro.data = omega_;
      RCLCPP_INFO(this->get_logger(), "omega_Gyro: '%f'", omega_Gyro.data);

      publisher_1_->publish(omega_Gyro);

    }

    void update(double omega_gyro)
    {
      auto omega_Gyro = std_msgs::msg::Float64();
      omega_Gyro.data = omega_gyro;

      publisher_1_->publish(omega_Gyro);
      RCLCPP_INFO(this->get_logger(), "omega_Gyro: '%f'", omega_Gyro.data);

    }

    void update_callback()
    {
    update(omega_);
    }


    // ROS timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_1_;

    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_1_;

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

   







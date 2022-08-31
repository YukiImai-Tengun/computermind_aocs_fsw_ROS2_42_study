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
    : Node("Star_Tracker_sensor")
    {
      // Subscriber作成　キュー10個
      // Subscribe時にコールバック関数"topic_callback_1"を呼ぶ
      subscription_1_ = this->create_subscription<std_msgs::msg::Float64>(
      "theta_raw", 10, std::bind(&MinimalPubSub::topic_callback_1, this, _1));

      // Publisher作成
      publisher_1_ = this->create_publisher<std_msgs::msg::Float64>("theta_ST", 0);

      // 
      update_timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalPubSub::update_callback, this));

    }
 

  private:
    double theta_ = 0.0;

    void topic_callback_1(const std_msgs::msg::Float64::SharedPtr msg)
    {

      //subscribeした値でglobal変数を更新
      theta_ = msg->data;

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
      #if 1
      std::ofstream writing_file;
      std::string filename = "Star_Tracker_theta_raw_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(theta_) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////



      auto theta_ST = std_msgs::msg::Float64();
      theta_ST.data = theta_;
      RCLCPP_INFO(this->get_logger(), "theta_ST: '%f'", theta_ST.data);

      publisher_1_->publish(theta_ST);
    }



    void update(double theta_st)
    {
      auto theta_ST = std_msgs::msg::Float64();
      theta_ST.data = theta_st;

      publisher_1_->publish(theta_ST);
      RCLCPP_INFO(this->get_logger(), "theta_ST: '%f'", theta_ST.data);

    }

    void update_callback()
    {
    update(theta_);
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

   







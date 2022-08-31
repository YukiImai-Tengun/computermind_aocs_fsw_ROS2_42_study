// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <fstream>
#include <math.h>

// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"

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
    : Node("PD_controller")
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      subscription_1_ = this->create_subscription<std_msgs::msg::Float64>(
      "theta_est", 10, std::bind(&MinimalPubSub::topic_callback_1, this, _1));
      subscription_2_ = this->create_subscription<std_msgs::msg::Float64>(
      "omega_est", 10, std::bind(&MinimalPubSub::topic_callback_2, this, _1));
      subscription_3_ = this->create_subscription<std_msgs::msg::Float64>(
      "theta_tgt", 
      0, std::bind(&MinimalPubSub::topic_callback_3, this, _1));

      publisher_1_ = this->create_publisher<std_msgs::msg::Float64>("T_rw_command", 10);

      //update_timer_ = this->create_wall_timer(1ms, std::bind(&MinimalPubSub::update_callback, this));

    }
 

  private:

  double theta_est = 0.0;
  double omega_est = 0.0;
  double theta_tgt = 0.0;
  double theta_tgt_prev = 0.0;
  double theta_err = 0.0;
  double omega_err = 0.0;

  double K_p = 0.2; //0.08 // 5
  double K_d = 0.2; //0.1 // 100

  int count = 0;
  int count_prev = 0;

    // Receives the String message that is published over the topic
    void topic_callback_1(const std_msgs::msg::Float64::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard theta_est : '%f'", msg->data);
      theta_est = msg->data;
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
      
      #if 0
      // ファイル書き込み部分
      std::ofstream writing_file;
      std::string filename = "PD_controller_theta_est_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(theta_est) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////
      
    
      update();
    }

    void topic_callback_2(const std_msgs::msg::Float64::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard omega_est : '%f'", msg->data);
      omega_est = msg->data;

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
      std::string filename = "PD_controller_omega_est_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(omega_est) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////

      //update();
    }

    void topic_callback_3(const std_msgs::msg::Float64::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard theta_tgt : '%f'", msg->data);
      theta_tgt = msg->data;


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
      std::string filename = "PD_controller_theta_tgt_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////


      //std::cout << theta_tgt << " " << clk2->now().nanoseconds() << std::endl;


      //update();
    }


    void update()
    {

      auto T_rw = std_msgs::msg::Float64();

      theta_err = theta_tgt - theta_est;
      theta_err = fmod(theta_tgt - theta_est, 360);

      //omega_err = (theta_tgt - theta_tgt_prev) / 0.1 - omega_est; // 遅いとき: 1 Hz -> 1 ###
      omega_err = (theta_tgt - theta_tgt_prev) / 0.1 - omega_est; // 遅いとき: 1 Hz -> 1 ###

      T_rw.data = (theta_err * K_p + omega_err * K_d) * -1;

      //RCLCPP_INFO(this->get_logger(), "theta_tgt_prev: '%f'", theta_tgt_prev);
      //RCLCPP_INFO(this->get_logger(), "theta_tgt: '%f'", theta_tgt);
      RCLCPP_INFO(this->get_logger(), "sabun: '%f'", theta_tgt - theta_tgt_prev);

      RCLCPP_INFO(this->get_logger(), "omega_tgt: '%f'", (theta_tgt - theta_tgt_prev) / 0.1);
      RCLCPP_INFO(this->get_logger(), "omega_est: '%f'", omega_est);

      //RCLCPP_INFO(this->get_logger(), "omega_err: '%f'", omega_err);
      RCLCPP_INFO(this->get_logger(), "theta_err: '%f'", theta_tgt - theta_est);
      //RCLCPP_INFO(this->get_logger(), "theta_fmod: '%f'", fmod(theta_tgt - theta_est, 360));


      theta_tgt_prev = theta_tgt;



      publisher_1_->publish(T_rw);
      RCLCPP_INFO(this->get_logger(), "T_rw: '%f'", T_rw.data);

    }


    


    // ROS timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_1_;

    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_1_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_2_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_3_;
    
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

   







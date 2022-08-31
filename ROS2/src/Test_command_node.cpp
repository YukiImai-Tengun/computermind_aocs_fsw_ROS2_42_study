// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <math.h>
#include <fstream>
#include <sys/time.h>


// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"


// Built-in message type that will be used to publish data
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

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
      // Publisher publishes String messages to a topic named "addison". 
      // The size of the queue is 10 messages.
      //publisher_1 = this->create_publisher<std_msgs::msg::Float64>("theta_ST", 10);
      //publisher_2 = this->create_publisher<std_msgs::msg::Float64>("omega_Gyro", 10);
      //publisher_3 = this->create_publisher<std_msgs::msg::Float64>("theta_est", 10);
      //publisher_4 = this->create_publisher<std_msgs::msg::Float64>("omega_est", 10);
      //publisher_5 = this->create_publisher<std_msgs::msg::Float64>("theta_tgt", 10);
      //publisher_6 = this->create_publisher<std_msgs::msg::Float64>("Torque", 10);
      publisher_7 = this->create_publisher<std_msgs::msg::Float64>("T_rw_command", 10);

      // publish at x milliseconds intervals
      //timer_1 = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::timer_callback_1, this));
      //timer_2 = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback_2, this));
      //timer_3 = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::timer_callback_3, this));
      //timer_4 = this->create_wall_timer(1ms, std::bind(&MinimalPublisher::timer_callback_4, this));
      //timer_5 = this->create_wall_timer(1000ms, std::bind(&MinimalPublisher::timer_callback_5, this));
      timer_6 = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::timer_callback_6, this));

      clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    }
 
  private:

  double value = 0;
  std::string seq;

    // This method executes every 500 milliseconds
    void timer_callback_1()
    {
      // Create a new message of type 
      auto message = std_msgs::msg::Float64();
      // Set our message's data attribute 
      //const double pi = std::acos(-1);
      //message.data = std::sin(pi/180 * 10 * count_++);
      //message.data = 1;
      //int counts = 1;
      value = value + 360/10 * 1/1;
      //message.data = double(count_++) * 360/10 * 1/1;
      message.data = value;

      // Print every message to the terminal window
      // RCLCPP_INFO(this->get_logger(), "Publishing theta_ST: '%f'", message.data);
       
      // Publish the message to the topic named "addison"
      publisher_1->publish(message);

      //double dt = (ros_clock.now() - message->header.stamp).nanoseconds();
      //RCLCPP_INFO(this->get_logger(), "Publishing time: '%f'", dt);

    }

    void timer_callback_2()
    {
      // Create a new message of type 
      auto message = std_msgs::msg::Float64();
      // Set our message's data attribute 
      //const double pi = std::acos(-1);
      //message.data = std::sin(pi/180 * 10 * count_++);
      //message.data = 1;
       message.data =  360/10;

      // Print every message to the terminal window
      //RCLCPP_INFO(this->get_logger(), "Publishing omega_Gyro: '%f'", message.data);
       
      // Publish the message to the topic named "addison"
      publisher_2->publish(message);

    }

    void timer_callback_3()
    {
      // Create a new message of type 
      auto message = std_msgs::msg::Float64();

      message.data = 1;
 
      // Print every message to the terminal window
      //RCLCPP_INFO(this->get_logger(), "Publishing theta_est, omega_est: '%f'", message.data);
      //RCLCPP_INFO(this->get_logger(), "Publishing omega_est: '%f'", message.data);
       
      // Publish the message to the topic named "addison"
      publisher_3->publish(message);
      publisher_4->publish(message);

    }

    void timer_callback_4()
    {
      // Create a new message of type 
      auto message = std_msgs::msg::Float64();
      

      message.data = double(count_++) * 360/10 * 1/1000;
 
      // Print every message to the terminal window
      //RCLCPP_INFO(this->get_logger(), "Publishing theta_tgt: '%f'", message.data);

      // 時間取得部分
      //rclcpp::TimeSource ts(shared_from_this());
      //rclcpp::Clock::SharedPtr clk2 = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      //ts.attachClock(clk2);
      //rclcpp::Clock clk3;

      //RCLCPP_INFO(this->get_logger(), "clk2 time: %ld", clk2->now().nanoseconds()); // clk2はROS時間を使用し、clk3はデフォルトでシステム時間 しかし、sim時間をオンにしない限り、それらは常に同じになります
      //RCLCPP_INFO(this->get_logger(), "clk3 time: %ld", clk3.now().nanoseconds()); // 
      //RCLCPP_INFO(this->get_logger(), "this->now time: %ld", this->now().nanoseconds()); // 
      //RCLCPP_INFO(this->get_logger(), "CONTACT!!! time: %ld", std::chrono::system_clock::now()); // システム時間
      

      // ファイルへの書き込み部分
      std::ofstream writing_file;
      std::string filename = "a.txt";
      writing_file.open(filename, std::ios::app);
      std::string writing_text = std::to_string(message.data);
      writing_file << writing_text << std::endl;
      writing_file.close();


      //std::cout << "" << << " " << std::endl;
      
      //RCLCPP_INFO(this->get_logger(), "time: '%f'",  clk3.now().nanoseconds());

      //RCLCPP_INFO(this->get_logger(), "Publishing omega_est: '%f'", message.data);
      
      // Publish the message to the topic named "addison"
      publisher_5->publish(message);
      

    }


    void timer_callback_5()
    {
      // Create a new message of type 
      auto message = std_msgs::msg::Float64();
      

      message.data = 1;

      seq = "1";
      RCLCPP_INFO(this->get_logger(), seq);

  
      //publisher_6->publish(message);
      

    }


    void timer_callback_6()
    {
      // Create a new message of type 
      auto message = std_msgs::msg::Float64();
    
      message.data = -1;

      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);

  
      publisher_7->publish(message);
      

    }









    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::TimerBase::SharedPtr timer_3;
    rclcpp::TimerBase::SharedPtr timer_4;
    rclcpp::TimerBase::SharedPtr timer_5;
    rclcpp::TimerBase::SharedPtr timer_6;


    //rclcpp::TimeSource ts(shared_from_this());
    //rclcpp::Clock::SharedPtr clk2 = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    //ts.attachClock(clk2);
    //rclcpp::Clock clk3;
    //rclcpp::Clock ros_clock(RCL_ROS_TIME);
    //rclcpp::Clock ros_clock(rcl_clock_type_t RCL_ROS_TIME);
    //rclcpp::Time now = ros_clock.now();
    //rclcpp::Clock ros_clock(RCL_ROS_TIME);
  
    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_1;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_2;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_3;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_4;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_5;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_6;  
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_7;  

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

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
class Rigid_Body : public rclcpp::Node
{

  double M_Inertia = 0.1; // 0.1
  double M_Inertia = 100; // 0.1
  double Torque_ = 0.0;
  double omega_dot = 0.0;
  double omega_ = 0.0;
  double theta_ = 0.0;


  public:
    // Constructor
    // The name of the node is minimal_subscriber
    Rigid_Body()
    : Node("Rigid_body")
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      subscription_Torque = this->create_subscription<std_msgs::msg::Float64>(
      "Torque", 10, std::bind(&Rigid_Body::callback_subscription_Torque, this, _1));
      publisher_omega_raw = this->create_publisher<std_msgs::msg::Float64>("omega_raw", 10);
      publisher_theta_raw = this->create_publisher<std_msgs::msg::Float64>("theta_raw", 10);

      // 1msの処理が終わらないうちに呼ばれたら？
      update_timer_ = this->create_wall_timer(10ms, std::bind(&Rigid_Body::update, this)); // 遅いとき: 100ms 

      // publishにはtimerはいらない？
      //omega_raw_publish_timer_ = this->create_wall_timer(10ms, std::bind(&Rigid_Body::callback_publisher_omega_raw, this));
      theta_raw_publish_timer_ = this->create_wall_timer(1000ms, std::bind(&Rigid_Body::callback_publisher_theta_raw, this)); // 遅いとき: 10s

    }
 

  private:

    void callback_subscription_Torque(const std_msgs::msg::Float64::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      Torque_ = msg->data;

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
      std::string filename = "Rigid_Body_Torque_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(Torque_) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////


    }


    void update()
    {
      auto omega_raw = std_msgs::msg::Float64();
      auto theta_raw = std_msgs::msg::Float64();

      omega_dot = Torque_ / M_Inertia;
      omega_ = omega_ + omega_dot * 0.01; // 遅いとき: 10 Hz -> 0.1
      theta_ = theta_ + omega_ * 0.01; // 遅いとき: 10 Hz -> 0.1
      
      omega_raw.data = omega_;
      publisher_omega_raw->publish(omega_raw);
      RCLCPP_INFO(this->get_logger(), "omega_raw: '%f'", omega_raw.data);
      //std::cout << theta_ << std::endl; 

      //theta_raw.data = theta_;
      //publisher_theta_raw->publish(theta_raw);
      //RCLCPP_INFO(this->get_logger(), "theta_raw: '%f'", theta_raw.data);

      //publisher_1_->publish(omega_raw);
      //RCLCPP_INFO(this->get_logger(), "omega_raw: '%f'", omega_raw.data);
      //publisher_2_->publish(theta_raw);
      //RCLCPP_INFO(this->get_logger(), "theta_raw: '%f'", theta_raw.data);
    }

    void callback_publisher_omega_raw()
    {
    auto omega_raw = std_msgs::msg::Float64();

    omega_raw.data = omega_;
    publisher_omega_raw->publish(omega_raw);
    
    RCLCPP_INFO(this->get_logger(), "omega_raw: '%f'", omega_raw.data);

    }

    void callback_publisher_theta_raw()
    {
    auto theta_raw = std_msgs::msg::Float64();

    theta_raw.data = theta_;
    publisher_theta_raw->publish(theta_raw);
    
    RCLCPP_INFO(this->get_logger(), "theta_raw: '%f'", theta_raw.data);

    }


    // ROS timer
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr omega_raw_publish_timer_;
    rclcpp::TimerBase::SharedPtr theta_raw_publish_timer_;

    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_omega_raw;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_theta_raw;
    

    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_Torque;
};



// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<Rigid_Body>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();

  return 0;
}

   







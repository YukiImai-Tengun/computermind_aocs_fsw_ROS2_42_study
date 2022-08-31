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
class KalmanFilter : public rclcpp::Node
{

  public:

    KalmanFilter()
    : Node("Kalman_filter")
    {
      // サブスクライバ作成
      subscription_theta_ST = this->create_subscription<std_msgs::msg::Float64>("theta_ST", 0, std::bind(&KalmanFilter::callback_subscription_theta_ST, this, _1));
      subscription_omega_Gyro = this->create_subscription<std_msgs::msg::Float64>("omega_Gyro", 0, std::bind(&KalmanFilter::callback_subscription_omega_Gyro, this, _1));
      
      // パブリッシャー作成
      publisher_theta_est = this->create_publisher<std_msgs::msg::Float64>("theta_est", 0);
      publisher_omega_est = this->create_publisher<std_msgs::msg::Float64>("omega_est", 0);

      // タイマー作成
      theta_est_publish_timer = this->create_wall_timer(100ms, std::bind(&KalmanFilter::callback_publisher_theta_est, this)); // 遅いとき: 1000 ms ###
      omega_est_publish_timer = this->create_wall_timer(100ms, std::bind(&KalmanFilter::callback_publisher_omega_est, this)); // 遅いとき: 1000 ms ###

    }
 

  private:

  std::mutex mtx;

  double theta_ST = 0.0;
  double omega_Gyro = 0.0;
  double theta_est_prev = 0.0;

  double theta_ = 0.0;
  double omega_ = 0.0;

  double time = 0;
  double time_prev = 0;

  bool flag = false;

  int count = 0;
  int count_prev = 0;

    // Receives the String message that is published over the topic

    void callback_subscription_theta_ST(const std_msgs::msg::Float64::SharedPtr msg)
    {

      //std::lock_guard lock(mtx);
      //std::lock_guard<std::mutex> lock(mtx);

      theta_ST =  msg->data;

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
      std::string filename = "Kalman_filter_theta_ST_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(theta_ST) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////

      //count++;
      flag = true;

      //update();
    }

    void callback_subscription_omega_Gyro(const std_msgs::msg::Float64::SharedPtr msg)
    {

      //std::lock_guard lock(mtx);
      //std::lock_guard<std::mutex> lock(mtx);

      omega_Gyro =  msg->data;

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
      std::string filename = "Kalman_filter_omega_Gyro_subscribe.txt";
      writing_file.open(filename, std::ios::app);
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(clk2->now().nanoseconds());
      //std::string writing_text = std::to_string(theta_tgt) + " " + std::to_string(this.get_clock().now().nanoseconds());
      std::string writing_text = std::to_string(omega_Gyro) + " " + std::to_string(this->now().nanoseconds());

      writing_file << writing_text << std::endl;
      writing_file.close();
      #endif
      ///////////////////////////////////////////////////////////////////////////////////

      double omega_local = omega_Gyro;

      //omega_ = omega_Gyro;
      //RCLCPP_INFO(this->get_logger(), "omega_local subscription: '%f'", omega_local);


      update();
    }

    void update()
    {

      //std::lock_guard<std::mutex> lock(mtx);

      if (flag == true) {
          RCLCPP_INFO(this->get_logger(), "IF");

          //RCLCPP_INFO(this->get_logger(), "flag: '%d'", flag);

          omega_ = omega_Gyro;
          theta_ = theta_ST;

          RCLCPP_INFO(this->get_logger(), "theta_: '%f'", theta_);

          RCLCPP_INFO(this->get_logger(), "omega_Gyro update() IF: '%f'", omega_Gyro);


          //RCLCPP_INFO(this->get_logger(), "omega_est SYNC: '%f'", omega_est.data);
          //RCLCPP_INFO(this->get_logger(), "theta_est SYNC: '%f'", theta_est.data);

          //count = 0;
          flag = false;


        } else {
          RCLCPP_INFO(this->get_logger(), "ELSE");

          double omega_local = omega_Gyro;

          //RCLCPP_INFO(this->get_logger(), "flag: '%d'", flag);
          
          omega_ = omega_local;          
          //omega_ = omega_Gyro;
          //RCLCPP_INFO(this->get_logger(), "omega_local update(): '%f'", omega_local);

          RCLCPP_INFO(this->get_logger(), "omega_Gyro update() ELSE: '%f'", omega_Gyro);

          //RCLCPP_INFO(this->get_logger(), "theta_est_prev: '%f'", theta_est_prev);
          //RCLCPP_INFO(this->get_logger(), "omega_est NOT SYNC: '%f'", omega_est.data);

          //theta_ = theta_est_prev + omega_local * 1; // 遅いとき: 10 Hz -> 0.1 ###
          theta_ = theta_est_prev + omega_local * 0.1; // 遅いとき: 10 Hz -> 0.1 ###
          RCLCPP_INFO(this->get_logger(), "theta_: '%f'", theta_);
          

          //RCLCPP_INFO(this->get_logger(), "theta_est NOT SYNC: '%f'", theta_est.data);

          //theta_est_prev = theta_;

        }

      theta_est_prev = theta_;
      std::cout << theta_ << std::endl; 


      //RCLCPP_INFO(this->get_logger(), "theta_est : '%f'", theta_);

      //count_prev = count;
      //RCLCPP_INFO(this->get_logger(), "flag: '%d'", flag);

    }

    
    void callback_publisher_theta_est()
    {
      //std::lock_guard<std::mutex> lock(mtx);
      //std::lock_guard lock(mtx);

      auto theta_est = std_msgs::msg::Float64();

      theta_est.data = theta_;
      publisher_theta_est->publish(theta_est);
      //RCLCPP_INFO(this->get_logger(), "theta_est : '%f'", theta_);

      //RCLCPP_INFO(this->get_logger(), "theta_est: '%f'", theta_est.data);
      //RCLCPP_INFO(this->get_logger(), "theta_est publish: '%f'", theta_est.data);
  
    }

    void callback_publisher_omega_est()
    {
      //std::lock_guard<std::mutex> lock(mtx);
      //std::lock_guard lock(mtx);

      auto omega_est = std_msgs::msg::Float64();

      omega_est.data = omega_;
      publisher_omega_est->publish(omega_est);
      //RCLCPP_INFO(this->get_logger(), "omega_est: '%f'", omega_est.data);
      //RCLCPP_INFO(this->get_logger(), "omega_est publish: '%f'", omega_est.data);

    }


    // ROS timer
    rclcpp::TimerBase::SharedPtr theta_est_publish_timer;
    rclcpp::TimerBase::SharedPtr omega_est_publish_timer;

    // Declaration of the publisher_ attribute
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_theta_est;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_omega_est;

    // Declare the subscription attribute
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_theta_ST;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_omega_Gyro;

};
 


// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<KalmanFilter>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();

  return 0;
}

   







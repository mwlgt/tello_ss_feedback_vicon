#include <chrono>
#include <functional>
#include <memory>
#include <algorithm> // Required for std::clamp

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/logging.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class StateFeedbackController : public rclcpp::Node
{
  public:
    StateFeedbackController()
    : Node("pController")
    {
      pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vicon/droniel/droniel", 10, std::bind(&StateFeedbackController::topic_callback, this, _1));

      desired_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/droniel/tello/desired_pose", 10, std::bind(&StateFeedbackController::desired_pose_callback, this, _1));
      
      twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("droniel/tello/control/uaux", 10);
      enable_publisher_ = this->create_publisher<std_msgs::msg::String>("/droniel/tello/enable", 10);

      timer_ = this->create_wall_timer(1000ms, std::bind(&StateFeedbackController::CallbackT, this));
      timer_control_ = this->create_wall_timer(20ms, std::bind(&StateFeedbackController::ControllCallback, this));

    }

  private:

    double get_yaw(double qw, double qx, double qy, double qz) {
        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        return euler[2]; 
    }

    void topic_callback(const geometry_msgs::msg::PoseStamped & msg) {
      if(msg.pose.orientation.w == 0 && msg.pose.orientation.x == 0 && msg.pose.orientation.y == 0 && msg.pose.orientation.z == 1){
        return; 
      }
      curr_x = msg.pose.position.x;
      curr_y = msg.pose.position.y;
      curr_z = msg.pose.position.z;
      
      // Extract full Euler angles for the 6-DOF measurement vector[cite: 1, 2]
      Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
      Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      curr_roll = euler[0];
      curr_pitch = euler[1];
      curr_yaw = euler[2];
    }

    void desired_pose_callback(const geometry_msgs::msg::PoseStamped & msg) {
      des_x = msg.pose.position.x;
      des_y = msg.pose.position.y;
      des_z = msg.pose.position.z;
      des_yaw = get_yaw(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    }

    void CallbackT() {
      std_msgs::msg::String enable_msg;
      enable_msg.data = "True";
      enable_publisher_->publish(enable_msg);
    }

    void ControllCallback() {


      double ex = des_x - curr_x;
      double ey = des_y - curr_y;
      double ez = des_z - curr_z;
      double eYaw = des_yaw-curr_yaw;
      
        


      geometry_msgs::msg::Twist output;
      
      // Clamp output values to the -100 to 100 range expected by the Tello drone
      output.linear.x = std::clamp(-70*ey, -100.0, 100.0); 
      output.linear.y = std::clamp(70*ex, -100.0, 100.0); 
      output.linear.z = 0.0;//std::clamp(70*ez, -100.0, 100.0);
      output.angular.z = 0.0;//std::clamp(eYaw,-100.0,100.0);

      RCLCPP_INFO(this->get_logger(),"Error x: %f",ex);
      
      twist_publisher_->publish(output);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr enable_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_control_;

    double curr_x = 0.0, curr_y = 0.0, curr_z = 0.0;
    double curr_roll = 0.0, curr_pitch = 0.0, curr_yaw = 0.0;
    double des_x = 0.0, des_y = 0.0, des_z = 0.0, des_yaw = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateFeedbackController>());
  rclcpp::shutdown();
  return 0;
}
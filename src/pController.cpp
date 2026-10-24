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

      init_matrices();
    }

  private:
    void init_matrices() {
        double g = 9.81;
        double m = 0.068;
        double Ix = 0.0000686;
        double Iy = 0.000092;
        double Iz = 0.0001366;

        A = Eigen::MatrixXd::Zero(12, 12);
        A(0,1)=1; A(1,8)=g; A(2,3)=1; A(3,6)=-g; A(4,5)=1; 
        A(6,7)=1; A(8,9)=1; A(10,11)=1;

        B = Eigen::MatrixXd::Zero(12, 4);
        B(5,0)=1/m; B(7,1)=1/Ix; B(9,2)=1/Iy; B(11,3)=1/Iz;

        // Expanded to 6x12 to match the full 6-DOF VICON pose measurements[cite: 1, 2]
        C = Eigen::MatrixXd::Zero(6, 12);
        C(0,0) = 1;  // x
        C(1,2) = 1;  // y
        C(2,4) = 1;  // z
        C(3,6) = 1;  // roll (phi)
        C(4,8) = 1;  // pitch (theta)
        C(5,10) = 1; // yaw (psi)

        K = Eigen::MatrixXd(4, 12);
        K << 0, 0, 0, 0, 0, 0.122400, 0, 0, 0, 0, 0, 0,
             0, 0, 0, -0.0006293577982, 0, 0, 0.003087, 0.00093296, 0, 0, 0, 0,
             0, 0.0008440366972, 0, 0, 0, 0, 0, 0, 0.004140, 0.00125120, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00024588;

        L = Eigen::MatrixXd(12, 6);
        L << 9.00, 0, 0, 0, 0, 0,
             20.00, 0, 0, 0, 0, 0,
             0, 9.20, 0, 0, 0, 0,
             0, 20.91, 0, 0, 0, 0,
             0, 0, 9.40, 0, 0, 0,
             0, 0, 21.84, 0, 0, 0,
             0, 0, 0, 9.60, 0, 0,
             0, 0, 0, 22.79, 0, 0,
             0, 0, 0, 0, 9.80, 0,
             0, 0, 0, 0, 23.76, 0,
             0, 0, 0, 0, 0, 10.00,
             0, 0, 0, 0, 0, 24.75;

        K_bar = Eigen::MatrixXd(4, 4);
        K_bar << 0, 0, 0.122400, 0,
                 0, -0.0006293577982, 0, 0,
                 0.0008440366972, 0, 0, 0,
                 0, 0, 0, 0.00024588;

        x_hat = Eigen::VectorXd::Zero(12);
        u = Eigen::VectorXd::Zero(4);
    }

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

    Eigen::MatrixXd A, B, C, L, K, K_bar;
    Eigen::VectorXd x_hat, u;

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
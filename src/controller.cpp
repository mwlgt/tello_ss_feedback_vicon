#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <rclcpp/publisher_base.hpp>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace std::chrono_literals;

class DroneControllerNode : public rclcpp::Node
{
public:
    DroneControllerNode() : Node("drone_state_feedback_controller")
    {
        // Publishers
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("droniel/tello/control/uaux", 10);
        enable_publisher_ = this->create_publisher<std_msgs::msg::String>("/droniel/tello/enable", 10);
        estimation_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("droniel/tello/estimation",10);


        // Subscribers
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vicon/droniel/droniel", 10, std::bind(&DroneControllerNode::pose_callback, this, std::placeholders::_1));
            
        desired_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/droniel/tello/desired_pose", 10, std::bind(&DroneControllerNode::desired_pose_callback, this, std::placeholders::_1));

        // Initialize Matrices
        init_matrices();

        // Timers
        dt_ = 0.02; // 20ms
        timer_control_ = this->create_wall_timer(
            20ms, std::bind(&DroneControllerNode::control_loop, this));
            
        timer_enable_ = this->create_wall_timer(
            1000ms, std::bind(&DroneControllerNode::enable_callback, this));
    }

private:
    void init_matrices()
    {
        double m = 0.068;
        double Iz = 0.0001366;

        A_ = Eigen::MatrixXd::Zero(8, 8);
        A_(0, 1) = 1.0; A_(2, 3) = 1.0; A_(4, 5) = 1.0; A_(6, 7) = 1.0;

        B_ = Eigen::MatrixXd::Zero(8, 4);
        B_(1, 0) = 1.0 / m; B_(3, 1) = -1.0 / m; B_(5, 2) = 1.0 / m; B_(7, 3) = 1.0 / Iz;

        C_ = Eigen::MatrixXd::Zero(4, 8);
        C_(0, 0) = 1.0; C_(1, 2) = 1.0; C_(2, 4) = 1.0; C_(3, 6) = 1.0;

        K_ = Eigen::MatrixXd::Zero(4, 8);
        K_ << 0.4524,  0.1813, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000,  0.0000, 
               -0.0000, -0.0000,  -0.4524,  -0.1813,  0.0000,  0.0000,  0.0000,  0.0000, 
              -0.0000, -0.0000,  0.0000,  0.0000,  0.4524,  0.1813,  0.0000, -0.0000,
              -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.0009,  0.0004;

        Kb_ = Eigen::MatrixXd::Zero(4, 4);
        Kb_ << 0.0022,  0.0000,  0.0000,  0.0000,
                0.0000, -0.0022,  0.0000,  0.0000,
                0.0000,  0.0000, 0.0022,  0.0000,
                0.0000,  0.0000,  0.0000, 1.1004;
        Kb_ *= 1000.0;

        Eigen::MatrixXd L_temp(4, 8);
        L_temp << 45.2399,  1.8133, 0.0000,  0.0000,  0.0000, -0.0000,  0.0000,  0.0000,
                 0.0000,  -0.000, 45.2399,  1.8133,  0.0000, -0.0000,  0.0000,  0.0000,
                 -0.0000,  0.0000, -0.0000, -0.0000, 45.2399,  1.8133, -0.0000, -0.0000,
                  0.0000,  0.0000,  0.0000,  0.0000, -0.0000,  0.0000,  0.0909,  0.0036;
        L_ = L_temp.transpose();

        x_hat_ = Eigen::VectorXd::Zero(8);
        u_ = Eigen::VectorXd::Zero(4);
        y_ = Eigen::VectorXd::Zero(4);
        ref_ = Eigen::VectorXd::Zero(4);
        
        has_pose_ = false;
    }

    double get_yaw_from_quaternion(double x, double y, double z, double w)
    {
        Eigen::Quaterniond eq(w, x, y, z);
        Eigen::Vector3d euler = eq.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
        return euler[0]; // Yaw
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if(msg->pose.orientation.w == 0 && msg->pose.orientation.x == 0 && 
           msg->pose.orientation.y == 0 && msg->pose.orientation.z == 1){ 
            return; // Ignore bad readings 
        } 

        y_(0) = msg->pose.position.x;
        y_(1) = msg->pose.position.y;
        y_(2) = msg->pose.position.z;
        y_(3) = get_yaw_from_quaternion(
            msg->pose.orientation.x, msg->pose.orientation.y, 
            msg->pose.orientation.z, msg->pose.orientation.w);
            
        has_pose_ = true;
    }

    void desired_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        ref_(0) = msg->pose.position.x;
        ref_(1) = msg->pose.position.y;
        ref_(2) = msg->pose.position.z;
        ref_(3) = get_yaw_from_quaternion(
            msg->pose.orientation.x, msg->pose.orientation.y, 
            msg->pose.orientation.z, msg->pose.orientation.w);
    }

    void enable_callback()
    {
        std_msgs::msg::String enable_msg; 
        enable_msg.data = "True"; 
        enable_publisher_->publish(enable_msg); 
    }

    void control_loop()
    {
        if (!has_pose_) return;

        // Observer Continuous integration using Euler method
        Eigen::VectorXd y_hat = C_ * x_hat_;
        Eigen::VectorXd x_hat_dot = A_ * x_hat_ + B_ * u_ + L_ * (y_ - y_hat);
        
        x_hat_ += x_hat_dot * dt_;

        // Calculate and log estimated tracking error
        Eigen::VectorXd key_term = y_-y_hat;
        RCLCPP_INFO(this->get_logger(), "Key term -> X: %.3f, Y: %.3f, Z: %.3f, Yaw: %.3f", 
                    x_hat_(1), x_hat_(3), x_hat_(5), x_hat_(7));


        geometry_msgs::msg::PoseStamped estimation;

        estimation.pose.position.x = x_hat_(0);
        estimation.pose.position.y = x_hat_(2);
        estimation.pose.position.z = x_hat_(4);

        estimation_publisher_ -> publish(estimation);


        // State feedback control with feedforward reference
        u_ = -K_ * x_hat_ + Kb_ * ref_;

        // Saturate control signals to [-100, 100] %
        for (int i = 0; i < 4; ++i) {
            if (u_(i) > 100.0) u_(i) = 100.0;
            if (u_(i) < -100.0) u_(i) = -100.0;
        }

        // u_(0)=0.0;
        // u_(1)=0.0;
        // u_(2)=0.0;
        // u_(3)=0.0;
        

        // Output configuration mapping
        geometry_msgs::msg::Twist output;
        output.linear.x = u_(1)*100;
        output.linear.y = u_(0)*100;
        output.linear.z = u_(2)*0;
        output.angular.z = u_(3)*0;

        twist_publisher_->publish(output);
    }

    // ROS Nodes
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr enable_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimation_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_subscription_;
    
    rclcpp::TimerBase::SharedPtr timer_control_;
    rclcpp::TimerBase::SharedPtr timer_enable_;

    // Variables
    Eigen::MatrixXd A_, B_, C_, K_, Kb_, L_;
    Eigen::VectorXd x_hat_, u_, y_, ref_;
    double dt_;
    bool has_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControllerNode>());
    rclcpp::shutdown();
    return 0;
}
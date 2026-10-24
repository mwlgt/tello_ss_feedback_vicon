#include <chrono>
#include <functional>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <rclcpp/timer.hpp>
#include <string>
#include <eigen3/Eigen/Dense>   
#include <eigen3/Eigen/Geometry>



#include "math.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("steps"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/droniel/tello/desired_pose", 10);
      timer_ = this->create_wall_timer(
      5000ms, std::bind(&MinimalPublisher::timer_callback, this));

    }

  private:
    void timer_callback()
    {

        double desired_x = 0.0;
        double desired_y = 0.0;
        double desired_z = 0.0;


        if(count % 4 == 0){

          desired_x = 1.0;
          desired_y = 0.0;
          desired_z = 0.0;
        }
        else if(count%4==1){
          desired_x = 1.0;
          desired_y = 1.0;
          desired_z = 0.0;
        }
        else if(count%4==2){
          desired_x = 0.0;
          desired_y = 1.0;
          desired_z = 0.0;
        }
        else if(count%4==3){
          desired_x = 0.0;
          desired_y = 0.0;
          desired_z = 0.0;
        }

        double desired_rw = 0.0;
        double desired_rx = 0.0;
        double desired_ry = 0.0;
        double desired_rz = 0.0;

        geometry_msgs::msg::PoseStamped out_msg;
        out_msg.pose.position.x = desired_x;
        out_msg.pose.position.y = desired_y;
        out_msg.pose.position.z = desired_z;
        out_msg.pose.orientation.x = desired_rx;
        out_msg.pose.orientation.y = desired_ry;
        out_msg.pose.orientation.z = desired_rz;
        out_msg.pose.orientation.w = desired_rw;

        out_msg.header.frame_id = "world";
        
        publisher_->publish(out_msg);
        count++;
    }


    int count = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
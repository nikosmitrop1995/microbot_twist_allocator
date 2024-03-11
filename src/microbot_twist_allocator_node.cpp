#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TwistAllocator : public rclcpp::Node
{
  public:
    TwistAllocator(): Node("microbot_twist_allocator_node")
    {
      RCLCPP_INFO(this->get_logger(), "Start microbot_twist_allocator_node");
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",10,std::bind(&TwistAllocator::twist_callback, this, std::placeholders::_1)
      );
    }
    ~TwistAllocator()
    {
      RCLCPP_INFO(this->get_logger(), "BYE BYE!!!");
    }
    
  private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Linear Velocity on x-axis: %f", msg->linear.x);
      RCLCPP_INFO(this->get_logger(), "Angular Velocity on yaw: %f", msg->angular.z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistAllocator>());
  rclcpp::shutdown();
  return 0;
}
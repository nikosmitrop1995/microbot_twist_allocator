#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>

#define R 0.0598
#define L 0.109

class TwistAllocator : public rclcpp::Node
{
  public:
    TwistAllocator(): Node("microbot_twist_allocator_node")
    {
      RCLCPP_INFO(this->get_logger(), "Start microbot_twist_allocator_node");
      
      // Publishers
      publisher_left_wheel_vel_ = this->create_publisher<std_msgs::msg::Float32>("left_wheel_vel",10);
      publisher_right_wheel_vel_ = this->create_publisher<std_msgs::msg::Float32>("right_wheel_vel",10);
      
      // Subscriber
      subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
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
      publish_wheel_vel(msg->linear.x, msg->angular.z);
    }
    
    void publish_wheel_vel(const float& ux, const float& w) const
    {
      // Left wheel velocity
      // υL = (2 * ux + ω*L)/(2*R)
      // Right wheel velocity
      // υR = (2 * ux - ω*L)/(2*R)
      // ux is linear.x
      // ω is angular.z
      // L is the distance between the wheels
      // R is the radius of the wheel
      
      // Left wheel velocity
      float uL = ((2 * ux) + (w * L)) / (2 * R);
      // Right wheel velocity
      float uR = ((2 * ux) - (w * L)) / (2 * R);

      auto msg = std_msgs::msg::Float32();

      // Assign and publish left wheel velocity
      msg.data = uL;
      publisher_left_wheel_vel_->publish(msg);

      // Assign and publish right wheel velocity
      msg.data = uR;
      publisher_right_wheel_vel_->publish(msg);

    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_left_wheel_vel_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_right_wheel_vel_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistAllocator>());
  rclcpp::shutdown();
  return 0;
}
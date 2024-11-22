#include <rclcpp/rclcpp.hpp>  
#include <chrono>
#include <string>
#include <memory>
#include "robot_msg/msg/keyframe_info.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<robot_msg::msg::KeyframeInfo>(
        "info", 
        rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        // .best_effort()
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        );
      timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        std::cout << "public " << "\n";
        robot_msg::msg::KeyframeInfo info;
        publisher_->publish(info);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<robot_msg::msg::KeyframeInfo>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

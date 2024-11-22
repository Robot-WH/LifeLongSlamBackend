#include <rclcpp/rclcpp.hpp>  
#include <string>
#include <memory>
#include "robot_msg/msg/keyframe_info.hpp"

class SubNode : public rclcpp::Node
{
public:
    explicit SubNode(const std::string& node_name)
        :Node(node_name)
    {
          rclcpp::CallbackGroup::SharedPtr callback_group =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          auto sub1_obt = rclcpp::SubscriptionOptions();
          sub1_obt.callback_group = callback_group;
        // auto callback = [&](const std::shared_ptr<builtin_interfaces::msg::Time> timestamp){
        //     RCLCPP_INFO_STREAM(this->get_logger(),"current_time is "<<timestamp->sec<<":"<<timestamp->nanosec);
        // } ;
        // subscriber_ = create_subscription<builtin_interfaces::msg::Time>("current_time",rclcpp::SystemDefaultsQoS(),
        //                 callback);
        // subscriber_ = create_subscription<builtin_interfaces::msg::Time>("current_time",rclcpp::SystemDefaultsQoS(),
        //         std::bind(&SubNode::callback, this, std::placeholders::_1));
            // subscriber_ = this->create_subscription<builtin_interfaces::msg::Time>(
            //     "current_time",
            //     rclcpp::SystemDefaultsQoS(),
            //     [this](const std::shared_ptr<builtin_interfaces::msg::Time> msg) {
            //         this->callback(msg);
            //     }
            // );
            subscriber_ = create_subscription<builtin_interfaces::msg::Time>(
                "current_time",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&SubNode::callback, this,
                    std::placeholders::_1),
                sub1_obt
            );

            subscriber2_ = create_subscription<robot_msg::msg::KeyframeInfo>(
                "info",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&SubNode::callback2, this,
                    std::placeholders::_1),
                sub1_obt
            );
    }

private:
    void callback(const builtin_interfaces::msg::Time::SharedPtr timestamp) {

    }

    void callback2(const robot_msg::msg::KeyframeInfo::SharedPtr msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "receive keyframe");
    }
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subscriber_;
    rclcpp::Subscription<robot_msg::msg::KeyframeInfo>::SharedPtr subscriber2_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SubNode>("topic_sub");

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}

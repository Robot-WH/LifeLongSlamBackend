#include <rclcpp/rclcpp.hpp>  
#include <rclcpp/logging.hpp>
#include <string>
#include <memory>
#include "robot_msg/msg/keyframe_info.hpp"

class Comm
{
public:
    Comm() {
          node_ = rclcpp::Node::make_shared("comm");
          callback_group_  =
                node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          rclcpp::CallbackGroup::SharedPtr callback_group  =
                node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          auto sub1_obt = rclcpp::SubscriptionOptions();
        //   sub1_obt.callback_group = callback_group_;
        sub1_obt.callback_group = callback_group;
        // auto callback = [&](const std::shared_ptr<builtin_interfaces::msg::Time> timestamp){
        //     RCLCPP_INFO_STREAM(this->get_logger(),"current_time is "<<timestamp->sec<<":"<<timestamp->nanosec);
        // } ;
        // subscriber_ = create_subscription<builtin_interfaces::msg::Time>("current_time",rclcpp::SystemDefaultsQoS(),
        //                 callback);
        // subscriber_ = create_subscription<builtin_interfaces::msg::Time>("current_time",rclcpp::SystemDefaultsQoS(),
        //         std::bind(&SubNode::callback, this, std::placeholders::_1));
            // subscriber_ = node_->create_subscription<builtin_interfaces::msg::Time>(
            //     "current_time",
            //     rclcpp::SystemDefaultsQoS(),
            //     std::bind(&Comm::callback, this,
            //         std::placeholders::_1),
            //     sub1_obt
            // );
            subscriber_ = node_->create_subscription<builtin_interfaces::msg::Time>(
                "current_time",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&Comm::callback, this,
                    std::placeholders::_1)
            );

            subscriber2_ = node_->create_subscription<robot_msg::msg::KeyframeInfo>(
                "info",
                // rclcpp::SystemDefaultsQoS(),
                rclcpp::QoS(rclcpp::KeepLast(1))    // 历史策略：keep last
                // rclcpp::QoS(rclcpp::KeepAll())   //  历史策略：keep all
                // .reliable()      // 可靠性   
                .best_effort()  // 可靠性
                .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE),
                // .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),    // 持久性
                std::bind(&Comm::callback2, this, std::placeholders::_1),
                sub1_obt
            );
            executor_.add_node(node_);
    }

    void Run() {
         executor_.spin();
    }

private:
    void callback(const builtin_interfaces::msg::Time::SharedPtr timestamp) {
    }
    void callback2(const robot_msg::msg::KeyframeInfo::SharedPtr msg) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "receive keyframe");
    }
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subscriber_;
    rclcpp::Subscription<robot_msg::msg::KeyframeInfo>::SharedPtr subscriber2_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    Comm comm;  
    comm.Run();   
    rclcpp::shutdown();
    return 0;
}


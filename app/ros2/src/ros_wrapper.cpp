/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: lwh
 * @Version: 1.0
 * @Description:  长期建图定位后端  ros2接口  
 * @Others: 
 */
#include <deque>
#include <mutex>
#include <dirent.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 

#include <rclcpp/rclcpp.hpp>  

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
// #include "ros_utils.hpp"
#include "lifelong_backend/backend_lifelong.h"
#include "lifelong_backend/InnerComm/InnerProcessComm.hpp"
#include "robot_msg/msg/keyframe_info.hpp"
#include "robot_msg/msg/workspace_info.hpp"
#include "robot_msg/srv/save_data.hpp"
#include "robot_msg/srv/save_map.hpp"
#include "robot_msg/srv/save_traj.hpp"
#include "robot_msg/srv/set_command.hpp"
#include "robot_msg/srv/set_space.hpp"
#include "robot_msg/srv/set_traj.hpp"

using PointT = pcl::PointXYZI;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;  
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;  
using PointCloud = pcl::PointCloud<PointT>;
using UsedPointT = PointXYZIRDTC;                                     

class RosWrapper : public rclcpp::Node {
public:  
    RosWrapper() : Node("backend_node") {  
        // 发布者
        // Qos可靠，不需要持久
        rclcpp::QoS qos_pub(rclcpp::KeepLast(1));    // 历史策略：keep last        rclcpp::KeepAll()
        qos_pub.reliable();      // 可靠性    best_effort() 
        qos_pub.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);  // 不需要持久化
        workspace_pub_ = this->create_publisher<robot_msg::msg::WorkspaceInfo>("/workspace", qos_pub);  
        odom_to_map_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_to_map", qos_pub); 
        localize_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localize_map", qos_pub); 
        global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", qos_pub); 
        // 服务
        save_data_server_ = this->create_service<robot_msg::srv::SaveData>("/SaveData",
            std::bind(&RosWrapper::SaveDataService, this, std::placeholders::_1, std::placeholders::_2));
        save_map_server_ = this->create_service<robot_msg::srv::SaveMap>("/SaveMap",
            std::bind(&RosWrapper::SaveMapService, this, std::placeholders::_1, std::placeholders::_2));
        save_traj_server_ = this->create_service<robot_msg::srv::SaveTraj>("/SaveTraj",
            std::bind(&RosWrapper::SaveTrajService, this, std::placeholders::_1, std::placeholders::_2));
        set_space_server_ = this->create_service<robot_msg::srv::SetSpace>("/SetSpace",
            std::bind(&RosWrapper::SetSpaceService, this, std::placeholders::_1, std::placeholders::_2));
        set_traj_server_ = this->create_service<robot_msg::srv::SetTraj>("/SetTraj",
            std::bind(&RosWrapper::SetTrajService, this, std::placeholders::_1, std::placeholders::_2));
        set_workMode_server_ = this->create_service<robot_msg::srv::SetCommand>("/SetWorkMode",
            std::bind(&RosWrapper::SetWorkMode, this, std::placeholders::_1, std::placeholders::_2));
        // 订阅者
        // 订阅回调分组
        rclcpp::CallbackGroup::SharedPtr  callback_group  =
                this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group;

        keyframe_sub_ = this->create_subscription<robot_msg::msg::KeyframeInfo>(
                "/keyframe_info", // 话题名称
                rclcpp::QoS(rclcpp::KeepLast(10)).
                reliable().
                durability(RMW_QOS_POLICY_DURABILITY_VOLATILE),     // RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                std::bind(&RosWrapper::keyframeCallback, this, std::placeholders::_1)
        );
        get_work_space_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/getWorkSpace", // 话题名称
                rclcpp::QoS(rclcpp::KeepLast(10)).
                reliable().
                durability(RMW_QOS_POLICY_DURABILITY_VOLATILE),     // RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                std::bind(&RosWrapper::getWorkSpaceCallback, this, std::placeholders::_1)
        );

        // 进程内通信
        IPC::Server::Instance().Subscribe("odom_to_map", &RosWrapper::pubOdomToMap, this);  
        IPC::Server::Instance().Subscribe("localize_map", &RosWrapper::pubLocalizeMap, this);   
        IPC::Server::Instance().Subscribe("global_map", &RosWrapper::pubGlobalMap, this);
        // 初始化
        std::string config_path;     // 算法参数文件path  
        backend_.reset(new lifelong_backend::LifeLongBackEndOptimization<PointT>(config_path));
    }
private:
     ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void keyframeCallback(const robot_msg::msg::KeyframeInfo::SharedPtr msg) {
        std::cout << "keyframeCallback" << std::endl;
        SlamLib::CloudContainer<PointT> lidar_data;
        /**
         * @brief 检查这个时间戳问题  
         * 
         */
        lidar_data.timestamp_start_ = msg->filtered_pointcloud.header.stamp.sec;
        lidar_data.pointcloud_data_["filtered"] = std::make_shared<pcl::PointCloud<PointT>>();
        // pcl::fromROSMsg(msg->filtered_pointcloud, *lidar_data.pointcloud_data_["filtered"]);

        Eigen::Isometry3d odom;
        Eigen::Quaterniond odom_rot(msg->odom.pose.pose.orientation.w,
                                    msg->odom.pose.pose.orientation.x,
                                    msg->odom.pose.pose.orientation.y,
                                    msg->odom.pose.pose.orientation.z);
        odom.linear() = odom_rot.toRotationMatrix();
        odom.translation() << msg->odom.pose.pose.position.x,
                             msg->odom.pose.pose.position.y,
                             msg->odom.pose.pose.position.z;
        
        backend_->AddKeyFrame(lidar_data, odom);  
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void getWorkSpaceCallback(const std_msgs::msg::Bool::SharedPtr flag) {
        std::cout << "getWorkSpaceCallback" << std::endl;
        // 读取数据库内所有地图空间的名字  并发送信息
        DIR* dir_p = nullptr;
        struct dirent* dir_entry = nullptr;
        
        if ((dir_p = opendir(database_path_.data())) == nullptr) {
            std::cout << "dataset_path error" << std::endl;
            return; 
        }

        std::vector<std::string> workspace; 

        while ((dir_entry = readdir(dir_p)) != nullptr) {
            // 排除隐藏文件
            if (dir_entry->d_name[0] != '.') {
                workspace.push_back(dir_entry->d_name);
            }
        }

        robot_msg::msg::WorkspaceInfo info;
        info.space_name = workspace;
        workspace_pub_->publish(info);  

        closedir(dir_p);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool SaveDataService(const robot_msg::srv::SaveData::Request::SharedPtr req, const robot_msg::srv::SaveData::Response::SharedPtr res) {
        std::string directory = req->destination;
        res->success = false;
        backend_->SavePoseGraph();  
        std::cout<<"Save Graph Data success! "<<std::endl;
        res->success = true; 
        return res->success;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool SaveMapService(const robot_msg::srv::SaveMap::Request::SharedPtr req, const robot_msg::srv::SaveMap::Response::SharedPtr res) {
        std::string directory = req->destination;
        res->success = false;
        // System->SaveGlobalMap(req.resolution, directory);  
        std::cout<<"SaveGlobalMap success! resolution: "<<req->resolution<<std::endl;
        res->success = true; 
        return res->success;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool SaveTrajService(const robot_msg::srv::SaveTraj::Request::SharedPtr req, const robot_msg::srv::SaveTraj::Response::SharedPtr res) {    
        backend_->SavePoseGraph();  
        res->traj_id = lifelong_backend::PoseGraphDataBase::GetInstance().GetTrajectoryIDList();  
        return true;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool SetSpaceService(const robot_msg::srv::SetSpace::Request::SharedPtr req, const robot_msg::srv::SetSpace::Response::SharedPtr res) {
        // 加载space    返回内部轨迹的数量
        res->traj_id = backend_->Load(database_path_ + req->space_name);
        return true;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool SetTrajService(const robot_msg::srv::SetTraj::Request::SharedPtr req, const robot_msg::srv::SetTraj::Response::SharedPtr res) {   
        res->success = backend_->SetTrajectory(req->traj_id); 
        return true;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool SetWorkMode(const robot_msg::srv::SetCommand::Request::SharedPtr req, const robot_msg::srv::SetCommand::Response::SharedPtr res) {   
        std::cout << "SetWorkMode" << std::endl;
        // 校验是否为设置模式命令
        res->success = 0; 
        if (req->type != 1) {
            return false;  
        }
        res->success = backend_->SetWorkMode(req->cmd);

        std::cout  << "res->success: " << (int)res->success << std::endl;
        return true;  
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void pubLocalizeMap(const pcl::PointCloud<PointT>::Ptr& map) {
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        // 检查是否有订阅者
        if (localize_map_pub_->get_subscription_count() > 0) {
            // 将PCL点云转换为ROS 2消息
            pcl::toROSMsg(*map, laserCloudTemp);
            // 设置时间戳（使用ROS 2的时间）
            laserCloudTemp.header.stamp = this->now();
            // 设置帧ID
            laserCloudTemp.header.frame_id = "map";
            // 发布消息
            localize_map_pub_->publish(laserCloudTemp);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void pubGlobalMap(const pcl::PointCloud<PointT>::Ptr& map) {
        std::cout << "-----------------------------------------------------------------pubGlobalMap" << std::endl;
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        if (global_map_pub_->get_subscription_count() != 0) {
            pcl::toROSMsg(*map, laserCloudTemp);
            rclcpp::Time current_time = now();   
            laserCloudTemp.header.stamp = current_time; // 转换为sensor_msgs/Time消息类型
            laserCloudTemp.header.frame_id = "map";
            global_map_pub_->publish(laserCloudTemp);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 
     * 
     * @param odom_to_map 
     */
    void pubOdomToMap(const Eigen::Isometry3d& odom_to_map) {
        trans_odom2map_ = odom_to_map;
        Eigen::Vector3d p = odom_to_map.translation();
        Eigen::Quaterniond quat(odom_to_map.rotation()); // 注意这里应该是rotation()而不是linear()
        quat.normalize();
        nav_msgs::msg::Odometry odom_to_map_msg;
        // 设置时间戳
        auto now = this->now();
        odom_to_map_msg.header.stamp = now;
        // 设置帧ID
        odom_to_map_msg.header.frame_id = "map";
        odom_to_map_msg.child_frame_id = "odom";
        // 设置位置
        odom_to_map_msg.pose.pose.position.x = p[0];
        odom_to_map_msg.pose.pose.position.y = p[1];
        odom_to_map_msg.pose.pose.position.z = p[2];
        // 设置四元数
        odom_to_map_msg.pose.pose.orientation.w = quat.w();
        odom_to_map_msg.pose.pose.orientation.x = quat.x();
        odom_to_map_msg.pose.pose.orientation.y = quat.y();
        odom_to_map_msg.pose.pose.orientation.z = quat.z();
        // 发布消息
        odom_to_map_pub_->publish(odom_to_map_msg);
    }
private:
    // 发布者成员变量  
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_to_map_pub_;  
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localize_map_pub_;  
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<robot_msg::msg::WorkspaceInfo>::SharedPtr workspace_pub_; 
    // 订阅者成员变量  
    rclcpp::Subscription<robot_msg::msg::KeyframeInfo>::SharedPtr keyframe_sub_; // 假设消息类型  
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr get_work_space_sub_; // 使用 String 作为示例  
    // 服务
    rclcpp::Service<robot_msg::srv::SaveData>::SharedPtr save_data_server_;
    rclcpp::Service<robot_msg::srv::SaveMap>::SharedPtr save_map_server_;
    rclcpp::Service<robot_msg::srv::SaveTraj>::SharedPtr save_traj_server_;
    rclcpp::Service<robot_msg::srv::SetSpace>::SharedPtr set_space_server_;   
    rclcpp::Service<robot_msg::srv::SetTraj>::SharedPtr set_traj_server_;   
    rclcpp::Service<robot_msg::srv::SetCommand>::SharedPtr set_workMode_server_;  
    Eigen::Isometry3d trans_odom2map_ = Eigen::Isometry3d::Identity();
    std::unique_ptr<lifelong_backend::LifeLongBackEndOptimization<PointT>> backend_;  

    std::string database_path_;                       
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    // rclcpp::spin(std::make_shared<RosWrapper>());  
    auto node = std::make_shared<RosWrapper>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();  
    return 0;
}

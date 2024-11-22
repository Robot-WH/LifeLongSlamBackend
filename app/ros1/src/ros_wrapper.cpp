/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: lwh
 * @Version: 1.0
 * @Description:  长期建图定位后端  ros1 接口  
 * @Others: 
 */
#include <deque>
#include <mutex>
#include <dirent.h>
#include <robot_msg/KeyframeInfo.h>
#include "ros_utils.hpp"
#include "lifelong_backend/backend_lifelong.h"
#include "lifelong_backend/InnerComm/InnerProcessComm.hpp"

using PointT = pcl::PointXYZI;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;  
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;  
using PointCloud = pcl::PointCloud<PointT>;  
// using UsedPointT = PointXYZIRDTC;

string config_path;     // 算法参数文件path  
string database_path;                                                                                      
string public_topic = "undistortion_pointcloud";
std::string odom_frame = "odom";

std::mutex m_estimate;
std::unique_ptr<lifelong_backend::LifeLongBackEndOptimization<PointT>> backend_;  

ros::Publisher pubUndistortPoints;  
std::vector<ros::Publisher> pubLidarFiltered;    // 发布每个激光滤波后的点   直接法时使用 
std::vector<ros::Publisher> pubLidarEdge;    // 发布每个激光提取的边缘特征
std::vector<ros::Publisher> pubLidarSurf;    // 发布每个激光提取的平面特征
std::vector<ros::Publisher> pubLocalMapFiltered;    // 发布每个激光滤波后的点   直接法时使用 
std::vector<ros::Publisher> pubLocalMapEdge;    // 发布每个激光提取的边缘特征
std::vector<ros::Publisher> pubLocalMapSurf;    // 发布每个激光提取的平面特征
ros::Publisher markers_pub; // 可视化
ros::Publisher odom_to_map_pub;   
ros::Publisher localizeMap_pub;  
ros::Publisher globalMap_pub;
ros::Publisher workspace_pub;  
ros::ServiceServer save_data_server;   // 数据保存服务
ros::ServiceServer save_map_server;  // 地图保存服务 
ros::ServiceServer set_space_server;  // 设置空间服务 
ros::ServiceServer save_traj_server; // 保存轨迹服务 
ros::ServiceServer set_traj_server; // 设置轨迹服务 
ros::ServiceServer set_workMode_server; // 设置工作模式服务  1\Lifelong  2\纯建图  3\纯定位 

ros::Subscriber keyframe_sub; 
ros::Subscriber getWorkSpace_sub; 
// 发布话题名称  
std::vector<std::string> pubLidarFiltered_topic = { "filtered_lidar_0", "filtered_lidar_1"};
std::vector<std::string> pubLidarEdge_topic = { "lidar_edge_0", "lidar_edge_1"};
std::vector<std::string> pubLidarSurf_topic = { "lidar_surf_0", "lidar_surf_1"};
std::vector<std::string> pubLocalMapFiltered_topic = { "LocalMap_filtered_0", "LocalMap_filtered_1"};
std::vector<std::string> pubLocalMapEdge_topic = { "LocalMap_edge_0", "LocalMap_edge_1"};
std::vector<std::string> pubLocalMapSurf_topic = { "LocalMap_surf_0", "LocalMap_surf_1"};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitSystem(ros::NodeHandle& n) {
    // 算法配置数据库路径
    config_path = RosReadParam<string>(n, "config_path");  
    backend_.reset(new lifelong_backend::LifeLongBackEndOptimization<PointT>(config_path));
    database_path = RosReadParam<string>(n, "database_path");
}

bool SaveDataService(lifelong_backend_package::SaveDataRequest& req, lifelong_backend_package::SaveDataResponse& res);
bool SaveMapService(lifelong_backend_package::SaveMapRequest& req, lifelong_backend_package::SaveMapResponse& res);
bool SetSpaceService(lifelong_backend_package::SetSpaceRequest& req, lifelong_backend_package::SetSpaceResponse& res);
bool SaveTrajService(lifelong_backend_package::SaveTrajRequest& req, lifelong_backend_package::SaveTrajResponse& res);
bool SetTrajService(lifelong_backend_package::SetTrajRequest& req, lifelong_backend_package::SetTrajResponse& res);
bool SetWorkMode(lifelong_backend_package::SetCommandRequest& req, lifelong_backend_package::SetCommandResponse& res);

void pubMarkers(const lifelong_backend::KeyFrameInfo<PointT>& info);
void keyframeCallback(const robot_msg::KeyframeInfo& info);
void getWorkSpaceCallback(const std_msgs::Bool& flag);
void pubOdomToMap(const Eigen::Isometry3d& odom_to_map);
void pubLocalizeMap(const pcl::PointCloud<PointT>::Ptr& map);     
void pubGlobalMap(const pcl::PointCloud<PointT>::Ptr& map);

void InitComm(ros::NodeHandle& private_nh, ros::NodeHandle& nh) {
    markers_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/graph_markers", 10);        // 可视化
    odom_to_map_pub = private_nh.advertise<nav_msgs::Odometry>("/odom_to_map", 10);   
    localizeMap_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/localize_map", 10);   
    globalMap_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);   
    workspace_pub = nh.advertise<lifelong_backend_package::workspace_info>("/workspace", 10);   
    save_data_server = private_nh.advertiseService("/SaveData", &SaveDataService);
    save_map_server = private_nh.advertiseService("/SaveMap", &SaveMapService);
    set_space_server = private_nh.advertiseService("/SetSpace", &SetSpaceService);
    save_traj_server = private_nh.advertiseService("/SaveTraj", &SaveTrajService);
    set_traj_server = private_nh.advertiseService("/SetTraj", &SetTrajService);
    set_workMode_server = private_nh.advertiseService("/SetWorkMode", &SetWorkMode);
    keyframe_sub = private_nh.subscribe("/keyframe_info", 1000, &keyframeCallback,
        ros::TransportHints().tcpNoDelay());
    getWorkSpace_sub = private_nh.subscribe("/getWorkSpace", 10, &getWorkSpaceCallback,
        ros::TransportHints().tcpNoDelay());
    // 进程内通信
    IPC::Server::Instance().Subscribe("keyframes_info", &pubMarkers);  
    IPC::Server::Instance().Subscribe("odom_to_map", &pubOdomToMap);  
    IPC::Server::Instance().Subscribe("localize_map", &pubLocalizeMap);   
    IPC::Server::Instance().Subscribe("global_map", &pubGlobalMap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SaveDataService(lifelong_backend_package::SaveDataRequest& req, lifelong_backend_package::SaveDataResponse& res) {
    std::string directory = req.destination;
    res.success = false;
    backend_->SavePoseGraph();  
    std::cout<<"Save Graph Data success! "<<std::endl;
    res.success = true; 
    return res.success;  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SaveMapService(lifelong_backend_package::SaveMapRequest& req, lifelong_backend_package::SaveMapResponse& res) {
    std::string directory = req.destination;
    res.success = false;
    // System->SaveGlobalMap(req.resolution, directory);  
    std::cout<<"SaveGlobalMap success! resolution: "<<req.resolution<<std::endl;
    res.success = true; 
    return res.success;  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SetSpaceService(lifelong_backend_package::SetSpaceRequest& req, lifelong_backend_package::SetSpaceResponse& res) {
    // 加载space    返回内部轨迹的数量
    res.traj_id = backend_->Load(database_path + req.space_name);
    return true;  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SaveTrajService(lifelong_backend_package::SaveTrajRequest& req, lifelong_backend_package::SaveTrajResponse& res) {
    backend_->SavePoseGraph();  
    res.traj_id = lifelong_backend::PoseGraphDataBase::GetInstance().GetTrajectoryIDList();  
    return true;  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SetTrajService(lifelong_backend_package::SetTrajRequest& req, lifelong_backend_package::SetTrajResponse& res) {
    res.success = backend_->SetTrajectory(req.traj_id); 
    return true;  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SetWorkMode(lifelong_backend_package::SetCommandRequest& req, lifelong_backend_package::SetCommandResponse& res) {
    std::cout << "SetWorkMode" << std::endl;
    // 校验是否为设置模式命令
    res.success = 0; 
    if (req.type != 1) {
        return false;  
    }
    res.success = backend_->SetWorkMode(req.cmd);

    std::cout  << "res.success: " << (int)res.success << std::endl;
    return true;  
}

Eigen::Isometry3d trans_odom2map = Eigen::Isometry3d::Identity();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getWorkSpaceCallback(const std_msgs::Bool& flag) {
    std::cout << "getWorkSpaceCallback" << std::endl;
    // 读取数据库内所有地图空间的名字  并发送信息
    DIR* dir_p = nullptr;
    struct dirent* dir_entry = nullptr;
    
    if ((dir_p = opendir(database_path.data())) == nullptr) {
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

    lifelong_backend_package::workspace_info info;
    info.space_name = workspace;
    workspace_pub.publish(info);  

    closedir(dir_p);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void keyframeCallback(const robot_msg::KeyframeInfo& info) {
    // std::cout << "keyframeCallback" << std::endl;
    SlamLib::CloudContainer<PointT> lidar_data;
    lidar_data.timestamp_start_ = info.filtered_pointcloud.header.stamp.toSec();
    lidar_data.pointcloud_data_.insert(std::make_pair("filtered", new pcl::PointCloud<PointT>));
    pcl::fromROSMsg<PointT>(info.filtered_pointcloud, *lidar_data.pointcloud_data_["filtered"]);
    Eigen::Isometry3d odom;
    Eigen::Quaterniond odom_rot;
    odom_rot.w() = info.odom.pose.pose.orientation.w;
    odom_rot.x() = info.odom.pose.pose.orientation.x;
    odom_rot.y() = info.odom.pose.pose.orientation.y;
    odom_rot.z() = info.odom.pose.pose.orientation.z;
    odom.linear() = odom_rot.toRotationMatrix(); 
    odom.translation().x()= info.odom.pose.pose.position.x;
    odom.translation().y()= info.odom.pose.pose.position.y;
    odom.translation().z()= info.odom.pose.pose.position.z;
    
    backend_->AddKeyFrame(lidar_data, odom);  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 根据图数据 创建可视化的图结构 
 */
template<typename _PointT>
visualization_msgs::MarkerArray createMarkerArray(lifelong_backend::KeyFrameInfo<PointT> const& info) {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(4);
    ros::Time stamp = ros::Time::now();  
    // node markers    位姿节点
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    // std::cout<<"info.vertex_database_.size(): "<<info.vertex_database_.size()<<std::endl;
    // std::cout<<"info.edge_database_.size(): "<<info.edge_database_.size()<<std::endl;
    // std::cout<<"info.new_keyframes_.size(): "<<info.new_keyframes_.size()<<std::endl;
    // 数量
    if (!info.localization_keyframe_pose_.empty()) {
        traj_marker.points.resize(info.localization_keyframe_pose_.size());
        // 颜色
        traj_marker.colors.resize(info.localization_keyframe_pose_.size());
        // 新增位姿节点
        for(int i=0; i<info.localization_keyframe_pose_.size(); i++) {
            // 设置位置
            Eigen::Vector3d pos = info.localization_keyframe_pose_[i].translation();
            traj_marker.points[i].x = pos.x();
            traj_marker.points[i].y = pos.y();
            traj_marker.points[i].z = pos.z();
            // 颜色
            traj_marker.colors[i].r = 0.0;
            traj_marker.colors[i].g = 1.0;
            traj_marker.colors[i].b = 1.0;
            traj_marker.colors[i].a = 1.0;
        }    
        return markers; 
    }

    traj_marker.points.resize(info.vertex_database_.size() + 
                                                            info.new_keyframes_.size());
    // 颜色
    traj_marker.colors.resize(info.vertex_database_.size() + 
                                                            info.new_keyframes_.size());
    // 新增位姿节点
    for(int i=0; i<info.new_keyframes_.size(); i++) {
        // 设置位置
        Eigen::Vector3d pos = (trans_odom2map * info.new_keyframes_[i].odom_).translation();
        traj_marker.points[i].x = pos.x();
        traj_marker.points[i].y = pos.y();
        traj_marker.points[i].z = pos.z();
        // 颜色
        traj_marker.colors[i].r = 1.0;
        traj_marker.colors[i].g = 0;
        traj_marker.colors[i].b = 0.0;
        traj_marker.colors[i].a = 1.0;
    }   
    // 关键帧数据库的位姿节点 
    for (int i = 0; i < info.vertex_database_.size(); i++) {
        // 设置位置
        Eigen::Vector3d pos = info.vertex_database_[i].pose_.translation();
        traj_marker.points[info.new_keyframes_.size() + i].x = pos.x();
        traj_marker.points[info.new_keyframes_.size() + i].y = pos.y();
        traj_marker.points[info.new_keyframes_.size() + i].z = pos.z();
        // 颜色
        traj_marker.colors[info.new_keyframes_.size()+i].r = 0;
        traj_marker.colors[info.new_keyframes_.size()+i].g = 0;
        traj_marker.colors[info.new_keyframes_.size()+i].b = 1.0;
        traj_marker.colors[info.new_keyframes_.size()+i].a = 1.0;
    }   
    // edge markers  边
    visualization_msgs::Marker& edge_marker = markers.markers[2];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 2;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.1;
    // 这里要注意 ！！！！
    edge_marker.points.resize(info.edge_database_.size() * 2); 
    edge_marker.colors.resize(info.edge_database_.size() * 2); 
    int i=0;

    for (int num = 0; num < info.edge_database_.size(); num++) {
        uint64_t local_id = info.edge_database_[num].link_head_local_index_; 
        Eigen::Vector3d pt1 = 
            info.vertex_database_[local_id].pose_.translation();
        // Twc*Tlc^-1 = Twl
        Eigen::Vector3d pt2 = 
            (info.vertex_database_[local_id].pose_
                * info.edge_database_[num].constraint_).translation();
        // 设置位置关系     每个frame 2个点 
        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].r = 1.0;
        edge_marker.colors[i*2].g = 2.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
        edge_marker.colors[i*2 + 1].g = 2.0;
        edge_marker.colors[i*2 + 1].a = 1.0;
        i++;
        
        // if(enable_GNSS_optimize)
        // {
        //     // GNSS 先验边     2个点  
        //     if(keyframes[num]->GNSS_Valid) {
        //         Eigen::Vector3d pt1 = keyframes[num]->Pose.translation();  
        //         Eigen::Vector3d pt2 = keyframes[num]->utm_coord;

        //         edge_marker.points[i*2].x = pt1.x();
        //         edge_marker.points[i*2].y = pt1.y();
        //         edge_marker.points[i*2].z = pt1.z()+1;
        //         edge_marker.points[i*2 + 1].x = pt2.x();
        //         edge_marker.points[i*2 + 1].y = pt2.y();
        //         edge_marker.points[i*2 + 1].z = pt2.z();

        //         edge_marker.colors[i*2].r = 1.0;
        //         edge_marker.colors[i*2].a = 1.0;
        //         edge_marker.colors[i*2 + 1].r = 1.0;
        //         edge_marker.colors[i*2 + 1].a = 1.0;
        //         i++;
        //     }
        // }
        // 地面约束边  
        // if (enable_planeConstraint_optimize)
        // {
        //     if (keyframe_database[num]->planeConstraint_Valid) 
        //     {
        //         Eigen::Vector3d plane = {pt1[0], pt1[1], 0};

        //         edge_marker.points[i*2].x = pt1.x();
        //         edge_marker.points[i*2].y = pt1.y();
        //         edge_marker.points[i*2].z = pt1.z();
        //         edge_marker.points[i*2 + 1].x = plane.x();
        //         edge_marker.points[i*2 + 1].y = plane.y();
        //         edge_marker.points[i*2 + 1].z = plane.z()-1;

        //         edge_marker.colors[i*2].r = 1.0;
        //         edge_marker.colors[i*2].a = 2.0;
        //         edge_marker.colors[i*2 + 1].r = 1.0;
        //         edge_marker.colors[i*2 + 1].a = 2.0;
        //         i++;
        //     }
        // }
    }
    // // 回环检测的边   2个点 
    // for (auto const& loop:info.loops_) { 
    //     Eigen::Vector3d pt1 = info.keyframe_database_[loop.id_1_].correct_pose_.translation(); // 新帧  
    //     Eigen::Vector3d pt2 = (info.keyframe_database_[loop.id_1_].correct_pose_
    //                                                     * loop.relative_pose_).translation();     // 与新帧闭环的老帧   Twc * Tlc^-1

    //     edge_marker.points[i*2].x = pt1.x();
    //     edge_marker.points[i*2].y = pt1.y();
    //     edge_marker.points[i*2].z = pt1.z();
    //     edge_marker.points[i*2 + 1].x = pt2.x();
    //     edge_marker.points[i*2 + 1].y = pt2.y();
    //     edge_marker.points[i*2 + 1].z = pt2.z();

    //     edge_marker.colors[i*2].r = 2.0;
    //     edge_marker.colors[i*2].a = 2.0;
    //     edge_marker.colors[i*2 + 1].r = 2.0;
    //     edge_marker.colors[i*2 + 1].a = 2.0;
    //     i++;
    // }
    // sphere
    visualization_msgs::Marker& sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "Odom Error";
    sphere_marker.id = 3;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    
    if (!info.new_keyframes_.empty()) {
        Eigen::Vector3d pos = (trans_odom2map * info.new_keyframes_.back().odom_).translation();
        sphere_marker.pose.position.x = pos.x();
        sphere_marker.pose.position.y = pos.y();
        sphere_marker.pose.position.z = pos.z();

        sphere_marker.pose.orientation.w = 1.0;
        sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 100;

        sphere_marker.color.r = 1.0;
        sphere_marker.color.a = 0.3;
    }

    return markers;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pubMarkers(const lifelong_backend::KeyFrameInfo<PointT>& info) {
    // 图优化可视化
    if (markers_pub.getNumSubscribers()) {
        // 发布 markers
        auto markers = createMarkerArray<PointT>(info);
        markers_pub.publish(markers);
    }  
}   

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pubOdomToMap(const Eigen::Isometry3d& odom_to_map) {
    trans_odom2map = odom_to_map;

    nav_msgs::Odometry odom_to_map_msg;
    Eigen::Vector3d p = odom_to_map.translation();
    Eigen::Quaterniond quat(odom_to_map.linear());
    quat.normalize();

    odom_to_map_msg.header.stamp = ros::Time::now();
    odom_to_map_msg.header.frame_id = "map";   
    odom_to_map_msg.child_frame_id = "odom";      
    odom_to_map_msg.pose.pose.position.x = p[0];
    odom_to_map_msg.pose.pose.position.y = p[1];
    odom_to_map_msg.pose.pose.position.z = p[2];
    // std::cout << "pubOdomToMap, x: " << p[0] << ",y: " << p[1] << ",z: " << p[2] << "\n";
    // 构造四元数   
    odom_to_map_msg.pose.pose.orientation.w = quat.w();
    odom_to_map_msg.pose.pose.orientation.x = quat.x();
    odom_to_map_msg.pose.pose.orientation.y = quat.y();
    odom_to_map_msg.pose.pose.orientation.z = quat.z();
    odom_to_map_pub.publish(odom_to_map_msg);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pubLocalizeMap(const pcl::PointCloud<PointT>::Ptr& map) {
    sensor_msgs::PointCloud2 laserCloudTemp;
    if (localizeMap_pub.getNumSubscribers() != 0) {
        pcl::toROSMsg(*map, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();;
        laserCloudTemp.header.frame_id = "map";
        localizeMap_pub.publish(laserCloudTemp);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pubGlobalMap(const pcl::PointCloud<PointT>::Ptr& map) {
    std::cout << "-----------------------------------------------------------------pubGlobalMap" << std::endl;
    sensor_msgs::PointCloud2 laserCloudTemp;
    if (globalMap_pub.getNumSubscribers() != 0) {
        pcl::toROSMsg(*map, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();;
        laserCloudTemp.header.frame_id = "map";
        globalMap_pub.publish(laserCloudTemp);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    ros::init(argc, argv, "lifelong_backend_package node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    InitComm(private_nh, nh);  
    InitSystem(nh);  
    ros::spin();
    return 0;
}
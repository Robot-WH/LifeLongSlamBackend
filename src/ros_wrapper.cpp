/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: lwh
 * @Version: 1.0
 * @Description:  长期建图定位后端  ros1 接口  
 * @Others: 
 */

#include "ros_utils.hpp"
#include "lifelong_backend/backend_lifelong.h"
#include "lifelong_backend/InnerComm/InnerProcessComm.hpp"
#include "lwio/keyframe_info.h"
#include <deque>
#include <mutex>

using PointT = pcl::PointXYZI;
using PointCloudConstPtr = pcl::PointCloud<PointT>::ConstPtr;  
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;  
using PointCloud = pcl::PointCloud<PointT>;  
// using UsedPointT = PointXYZIRDTC;

string config_path;     // 算法参数文件path                                                                                        
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
ros::ServiceServer save_data_server;   // 数据保存服务
ros::ServiceServer save_map_server;  // 地图保存服务 
ros::Subscriber keyframe_sub; 
// 发布话题名称  
std::vector<std::string> pubLidarFiltered_topic = { "filtered_lidar_0", "filtered_lidar_1"};
std::vector<std::string> pubLidarEdge_topic = { "lidar_edge_0", "lidar_edge_1"};
std::vector<std::string> pubLidarSurf_topic = { "lidar_surf_0", "lidar_surf_1"};
std::vector<std::string> pubLocalMapFiltered_topic = { "LocalMap_filtered_0", "LocalMap_filtered_1"};
std::vector<std::string> pubLocalMapEdge_topic = { "LocalMap_edge_0", "LocalMap_edge_1"};
std::vector<std::string> pubLocalMapSurf_topic = { "LocalMap_surf_0", "LocalMap_surf_1"};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitSystem(ros::NodeHandle &n) {
    // 算法配置文件路径
    config_path = RosReadParam<string>(n, "config_path");  
    backend_.reset(new lifelong_backend::LifeLongBackEndOptimization<PointT>(config_path));
}

bool SaveDataService(lifelong_backend::SaveDataRequest& req, lifelong_backend::SaveDataResponse& res);
bool SaveMapService(lifelong_backend::SaveMapRequest& req, lifelong_backend::SaveMapResponse& res);
void pubMarkers(const lifelong_backend::KeyFrameInfo<PointT>& info);
void keyframeCallback(const lwio::keyframe_info& info);
void pubOdomToMap(const Eigen::Isometry3d& odom_to_map);
void pubLocalizeMap(const pcl::PointCloud<PointT>::Ptr& map);  

void InitComm(ros::NodeHandle &private_nh) {
    // for (uint16_t i = 0; i < NUM_OF_LIDAR; i++)
    // {
    //     ros::Publisher pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarFiltered_topic[i], 10); 
    //     pubLidarFiltered.push_back(pub);  
    //     pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarSurf_topic[i], 10); 
    //     pubLidarSurf.push_back(pub);  
    //     pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLidarEdge_topic[i], 10); 
    //     pubLidarEdge.push_back(pub);  

    //     pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLocalMapFiltered_topic[i], 10); 
    //     pubLocalMapFiltered.push_back(pub);  
    //     pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLocalMapEdge_topic[i], 10); 
    //     pubLocalMapEdge.push_back(pub);  
    //     pub = private_nh.advertise<sensor_msgs::PointCloud2>(pubLocalMapSurf_topic[i], 10); 
    //     pubLocalMapSurf.push_back(pub);  
    // }
    markers_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/graph_markers", 10);        // 可视化
    odom_to_map_pub = private_nh.advertise<nav_msgs::Odometry>("/odom_to_map", 10);   
    localizeMap_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/localize_map", 10);   
    save_data_server = private_nh.advertiseService("/SaveGraph", &SaveDataService);
    save_map_server = private_nh.advertiseService("/SaveMap", &SaveMapService);
    keyframe_sub = private_nh.subscribe("/keyframe_info", 1000, &keyframeCallback,
        ros::TransportHints().tcpNoDelay());
    // 进程内通信
    IPC::Server::Instance().Subscribe("keyframes_info", &pubMarkers);  
    IPC::Server::Instance().Subscribe("odom_to_map", &pubOdomToMap);  
    IPC::Server::Instance().Subscribe("localize_map", &pubLocalizeMap);  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SaveDataService(lifelong_backend::SaveDataRequest& req, lifelong_backend::SaveDataResponse& res) {
    std::string directory = req.destination;
    // System->SavePoseGraph();  
    std::cout<<"Save Graph Data success! "<<std::endl;
    res.success = true; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SaveMapService(lifelong_backend::SaveMapRequest& req, lifelong_backend::SaveMapResponse& res) {
    std::string directory = req.destination;
    // System->SaveGlobalMap(req.resolution, directory);  
    std::cout<<"SaveGlobalMap success! resolution: "<<req.resolution<<std::endl;
    res.success = true; 
}


Eigen::Isometry3d trans_odom2map = Eigen::Isometry3d::Identity();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void keyframeCallback(const lwio::keyframe_info& info) {
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
        // 里程计边    Pc
        // Eigen::Vector3d pt1 = info.keyframe_database_[num].correct_pose_.translation();
        // // Twc*Tlc^-1 = Twl
        // Eigen::Vector3d pt2 = (info.keyframe_database_[num].correct_pose_ 
        //     * info.keyframe_database_[num].between_constraint_.inverse()).translation();
        Eigen::Vector3d pt1 = info.vertex_database_[info.edge_database_[num].link_id_.first].pose_.translation();
        // Twc*Tlc^-1 = Twl
        Eigen::Vector3d pt2 = (info.vertex_database_[info.edge_database_[num].link_id_.first].pose_
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
    // 回环检测的边   2个点 
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
    // 构造四元数   
    odom_to_map_msg.pose.pose.orientation.w = quat.w();
    odom_to_map_msg.pose.pose.orientation.x = quat.x();
    odom_to_map_msg.pose.pose.orientation.y = quat.y();
    odom_to_map_msg.pose.pose.orientation.z = quat.z();
    odom_to_map_pub.publish(odom_to_map_msg);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 发布定位点云可视化 
*/
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
int main(int argc, char **argv) {
    ros::init(argc, argv, "lifelong_backend node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    InitComm(private_nh);  
    InitSystem(nh);  
    // std::thread processResult_thread(processResult);
    ros::spin();
    return 0;
}




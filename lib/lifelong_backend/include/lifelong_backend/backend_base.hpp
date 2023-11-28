/**
 * @file backend_base.hpp
 * @author lwh 
 * @brief 
 * @version 0.1
 * @date 2023-06-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once 
#include <shared_mutex>
#include "SlamLib/Common/color.hpp"
#include "SlamLib/Common/point_type.h"
#include "Common/keyframe.hpp"
#include "pose_graph_database.hpp"
#include "LoopDetection/loopDetection.hpp"
namespace lifelong_backend {
template<typename _T>
struct KeyFrameInfo {
    double time_stamps_;  
    std::deque<KeyFrame> new_keyframes_;
    std::deque<Vertex> vertex_database_; 
    std::deque<Edge> edge_database_; 
};

template<typename _T>
struct LocalizationPointsInfo {
    double time_stamps_;  
    std::unordered_map<std::string, typename pcl::PointCloud<_T>::ConstPtr> scan_; 
    std::unordered_map<std::string, typename pcl::PointCloud<_T>::ConstPtr> map_;
};

template<typename _FeatureT>
class BackEndOptimizationBase {
public:
    virtual ~BackEndOptimizationBase() {}
    /**
     * @brief: 添加激光里程计数据  
     * @details: 
     */            
    virtual void AddKeyFrame(SlamLib::CloudContainer<_FeatureT> const& lidar_data, 
                                                                Eigen::Isometry3d const& odom) = 0; 
    virtual void Load() = 0; 
    virtual void SaveGlobalMap(float resolution, std::string save_path) = 0;  
    virtual void SavePoseGraph() = 0;  
    virtual void ForceGlobalOptimaze() = 0; 
protected:
    virtual void mapping() = 0;     // 处理线程
    virtual bool optimize() = 0;  

    using KeyFramePtr = typename KeyFrame::Ptr; 
    using KeyFrameConstPtr = typename KeyFrame::ConstPtr; 

    std::string keyframes_save_path_ = "/home/lwh/code/lwh_ws-master/src/liv_slam-master/Map";  
    // 新添加的关键帧的处理队列
    std::deque<KeyFrame> new_keyframe_queue_;
    std::deque<SlamLib::FeaturePointCloudContainer<_FeatureT>> new_keyframe_points_queue_;

    Eigen::Isometry3d trans_odom2map_ = Eigen::Isometry3d::Identity();
    std::mutex keyframe_queue_mutex_;
    std::shared_mutex keyframe_queue_sm_; 
    std::thread backend_thread_;
}; // class
} // namespace 

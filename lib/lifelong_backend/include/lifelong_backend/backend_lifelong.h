/*
 * @Copyright(C): 
 * @Author: lwh
 * @Version: 1.0
 * @Description: 对各种优化库进行了适配，通过多态进行优化库的切换
 * @Others: 
 */
#pragma once 
#include <stdexcept>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "backend_base.hpp"
#include "GraphOptimization/graph_optimization.h"
#include "GraphOptimization/graph_optimization_gtsam.h"
#include "GraphOptimization/graph_optimization_g2o.h"
// #include "Common/data_manager.hpp"
namespace lifelong_backend {
// using common::DataManager; 
#define debug 1
/**
 * @brief: 后端优化模块 , 优化器可选用gtsam/g2o 
 * @details: 实现 
 * 1、融合激光里程计约束，平面先验约束，gnss 约束的滑动窗口优化
 * 2、回环之后进行全局优化  
 * @param {*}
 * @return {*}
 */    
template<typename _FeatureT>
class LifeLongBackEndOptimization : public BackEndOptimizationBase<_FeatureT> {
private:
    struct Option {
        std::string database_save_path_;  
    } option_;
    enum class WorkMode {
        RELOCALIZATION = 0, // 开机之后处于该模式   接着会进行重定位找回定位 
        LOCALIZATION,
        MAPPING
    };
    // using base = BackEndOptimizationBase<_FeatureT>; 
    using PointCloudPtr = typename pcl::PointCloud<_FeatureT>::Ptr;  
    using PointCloudConstPtr = typename pcl::PointCloud<_FeatureT>::ConstPtr;  
    using FeaturePointCloudContainer = SlamLib::FeaturePointCloudContainer<_FeatureT>;
    using SourceT = std::pair<std::string, PointCloudConstPtr>;     // 匹配源类型    <id, 数据>
    using RegistrationPtr = typename SlamLib::pointcloud::RegistrationBase<_FeatureT>::Ptr;   
public:

    LifeLongBackEndOptimization(std::string config_path);

    /**
     * @brief: 外界调用的数据库载入接口 
     */            
    void Load() override;

    /**
     * @brief 
     * 
     * @param resolution 
     * @param save_path 
     */
    void SaveGlobalMap(float resolution, std::string save_path) override;  
    
    /**
     * @brief 
     * 
     */
    void SavePoseGraph() override;  

    /**
     * @brief: 添加激光里程计关键帧
     * @param lidar_data 激光雷达的数据  
     * @param odom 当前帧的前端里程计位姿 
     */            
    void AddKeyFrame(SlamLib::CloudContainer<_FeatureT> const& lidar_data, 
                                                Eigen::Isometry3d const& odom) override;

    // 强制执行一次全局优化   save的时候用
    void ForceGlobalOptimaze() override;

protected:

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 建图模式下将数据传输到其他模块
     * @param keyframe 关键帧数据
     * @param pointcloud_data 关键帧对应的点云
     */            
    void DataFurtherProcess(uint64_t const& id, SlamLib::FeaturePointCloudContainer<_FeatureT> pointcloud_data);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 定位线程
     * @details 1、通过位姿点云查找距离当前帧最近的历史关键帧
     *                      2、通过广度优先搜索查找节点的相邻帧，并拼接成local map
     *                      3、执行scan-map匹配，求取odom-map校正矩阵         
     *                      4、匹配评估，并根据结果进行模式切换
     */            
    void localization();

    /**
     * @brief: 后端建图线程  
     */            
    void mapping() override;

    /**
     * @brief: 对于新加入的关键帧new_keyframe_queue_，对每一个关键帧匹配约束，
     * @details: 约束对应好后放入 wait_optimize_keyframes_容器
     */            
    bool processData();

    /**
     * @brief: 全局pose graph优化
     * @details: 
     * @return 是否进行了优化 
     */            
    bool optimize() override;
private:
    // 局部优化每次处理的最大帧数  
    int max_keyframes_per_update_ = 10;
    int planeConstraint_optimize_freq_ = 5;
    bool enable_GNSS_optimize_ = false;  
    bool enable_planeConstraint_optimize_ = false;
    bool enable_map_update_ = true;  // 开启地图更新 
    bool has_loop_ = false;  
    uint64_t id_ = 0;
    uint64_t start_id_ = 0;
    uint16_t trajectory_ = 0;   

    // 回环模块 
    std::unique_ptr<LoopDetection<_FeatureT>> loop_detect_;  
    // 优化器
    std::unique_ptr<GraphOptimizerInterface> optimizer_;  
    // 定位匹配
    RegistrationPtr localize_registration_;
    // 匹配评估器
    SlamLib::pointcloud::PointCloudAlignmentEvaluate<_FeatureT> align_evaluator_;
    // 后端处理线程
    std::thread mapping_thread_;  
    std::thread localization_thread_;  
    uint16_t new_external_constranit_num_;  // 新加的外部约束数量(GPS，地面, IMU..)  决定是否要进行一次全局优化  
    WorkMode work_mode_; 
    Eigen::Isometry3d last_keyframe_odom_;  
    KeyFrame last_add_database_keyframe_;
}; // class 
} // namespace 

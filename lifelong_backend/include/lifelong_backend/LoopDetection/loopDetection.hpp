/**
 * @file loopDetection.hpp
 * @brief 闭环检测 - 基于激光描述子 + 几何位置    
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#include <thread>
#include <glog/logging.h>
#include "SlamLib/Common/pointcloud.h"
#include "SlamLib/Common/color.hpp"
#include "SlamLib/PointCloud/Registration/alignEvaluate.hpp"
#include "SlamLib/PointCloud/Registration/PCLRegistration.h"
#include "SlamLib/tic_toc.hpp"
#include "../pose_graph_database.hpp"
#include "../Common/keyframe.hpp"
#include "SceneRecognitionScanContext.hpp"
namespace lifelong_backend {
#define LOOP_DEBUG 0
struct LoopDetectionOption {
    double score_thresh;
    double overlap_thresh;
    double min_score;  
};
/**
 * @brief 重定位结果
 * 
 */
struct RelocResult {
    int32_t traj_id_ = 0;  
    uint32_t global_index_ = 0;
    Eigen::Isometry3d pose_;
};

/**
 * @brief: 基于雷达的闭环检测 - 1、先验位姿检测   2、点云描述子检测 
 *@param _PointType 回环时进行点云匹配的点类型 
    */    
template<typename _PointType>
class LoopDetection {   
private:
    using PointCloudPtr = typename pcl::PointCloud<_PointType>::Ptr;
    using PointCloudConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr;  
    using ConstFeaturePointcloudContainer = SlamLib::ConstFeaturePointCloudContainer<_PointType>;
    using FeaturePointcloudContainer = SlamLib::FeaturePointCloudContainer<_PointType>;
    using RegistrationPtr = std::unique_ptr<SlamLib::pointcloud::RegistrationBase<_PointType>>;   
    
    struct LoopKeyframeInfo {
        LoopKeyframeInfo(const uint32_t& id, const uint32_t& global_index, 
            const uint32_t& local_index) : global_index_(global_index), id_(id), local_index_(local_index) {}
        uint32_t id_;
        uint32_t global_index_;
        uint32_t local_index_;
    };
public:
    LoopDetection(LoopDetectionOption option) : SCORE_THRESH_(option.score_thresh), 
            OVERLAP_THRESH_(option.overlap_thresh), MIN_SCORE_(option.min_score) {
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>()); // 关键帧位姿管理树
        loop_thread_ = std::thread(&LoopDetection::LoopDetect, this);  
        // 创建NDT
        SlamLib::pointcloud::ndtomp_ptr<_PointType> rough_ndt_ptr = 
            SlamLib::pointcloud::CreateNDTOMP<_PointType>(3.0, 0.01, 0.3, 30, 4, "DIRECT1");  
        SlamLib::pointcloud::ndtomp_ptr<_PointType> refine_ndt_ptr = 
            SlamLib::pointcloud::CreateNDTOMP<_PointType>(1.0, 0.01, 0.1, 30, 4, "DIRECT7");  
        // 粗匹配算法初始化
        rough_registration_.reset(new SlamLib::pointcloud::PCLRegistration<_PointType>(
            std::move(rough_ndt_ptr), 
            "filtered"
        ));
        // 细匹配算法初始化
        refine_registration_.reset(new SlamLib::pointcloud::PCLRegistration<_PointType>(
            std::move(refine_ndt_ptr), 
            "filtered"
        ));
        // 获取 匹配算法所需要的点云名字
        rough_registration_specific_labels_ = rough_registration_->GetUsedPointsName();
        refine_registration_specific_labels_ = refine_registration_->GetUsedPointsName();
        evaluative_pointcloud_label_ = "filtered";
    }

    virtual ~LoopDetection() {}

    /**
     * @brief: 添加一个闭环检测的请求   
     * @param local_index 该关键帧结点处与位姿图的Index
     * @param scan_in 点云数据 
     * @return {*}
     */            
    void AddRequest(const uint32_t& id, const uint32_t& local_index, FeaturePointcloudContainer const& scan_in) {
        uint32_t global_index = scene_recognizer_.AddKeyFramePoints(scan_in);
        processed_queue_mt_.lock();
        wait_processed_keyframe_.emplace_back(id, global_index, local_index); 
        processed_queue_mt_.unlock();   
    }

    /**
     * @brief 
     * 
     * @param scan_in 
     * @return RelocResult 
     */
    RelocResult Relocalization(const FeaturePointcloudContainer& scan_in) {   
        SlamLib::time::TicToc tt; 
        // step1 先识别出相似帧，这里得到的是数据库的全局Index   
        std::pair<int64_t, Eigen::Isometry3d> res = scene_recognizer_.FindSimilarPointCloud(scan_in);  
        RelocResult reloc_res;
        if (res.first == -1) {
            reloc_res.traj_id_ = -1;
            return reloc_res; 
        }
        PoseGraphDataBase& poseGraph_database = PoseGraphDataBase::GetInstance(); 
        // 从数据库中读取该回环的vertex 信息
        Vertex loop_vertex = poseGraph_database.GetVertexByDatabaseIndex(res.first);  
        // std::cout << " curr_trajectory_id_: " << curr_trajectory_id_ << ",loop_vertex.traj_: "
        //         << loop_vertex.traj_ << ",loop_vertex.id_: " << loop_vertex.id_ << std::endl;
        // 通过kdtree 在同一条轨迹的历史结点中搜索若干个距离回环结点最接近的若干结点组成local map  
        pcl::PointXYZ loop_vertex_position; 
        loop_vertex_position.x = loop_vertex.pose_.translation().x();
        loop_vertex_position.y = loop_vertex.pose_.translation().y();
        loop_vertex_position.z = loop_vertex.pose_.translation().z();
        std::vector<int> search_ind;
        std::vector<float> search_dis;
        HistoricalPositionSearch(loop_vertex.traj_, loop_vertex_position, 20, 10, search_ind, search_dis);
        // step2 采用粗匹配 + 细匹配模式求解出位姿
        ConstFeaturePointcloudContainer localmaps;  
        // 将粗匹配所需要的点云local map 提取出来 
        for (auto const& name : rough_registration_specific_labels_) {   
            // 从数据库中查找 名字为 name 的点云 
            PointCloudPtr local_map(new pcl::PointCloud<_PointType>());
            if (!ConstructLocalmapByTrajectoryNode(loop_vertex.traj_, search_ind, name, local_map)) {
                LOG(WARNING) << SlamLib::color::RED << "Relocalization() error: can't find local map,name: "
                    << name << SlamLib::color::RESET;
                reloc_res.traj_id_ = -1;
                return reloc_res; 
            }
            rough_registration_->SetInputSource(std::make_pair(name, local_map)); 
            localmaps[name] = local_map; 
        }
        rough_registration_->SetInputTarget(scan_in);
        res.second = loop_vertex.pose_ * res.second;  
        // 进行粗匹配
        if (!rough_registration_->Solve(res.second)) {
            reloc_res.traj_id_ = -1;
            return reloc_res; 
        }
        // 细匹配
        // 将细匹配所需要的点云local map 提取出来 
        for (auto const& name : refine_registration_specific_labels_) {
            // 如果 细匹配所需要的local map 在之前粗匹配时  已经提取了
            if (localmaps.find(name) == localmaps.end()) {
                PointCloudPtr local_map(new pcl::PointCloud<_PointType>());
                if (!ConstructLocalmapByTrajectoryNode(loop_vertex.traj_, search_ind, name, local_map)) {
                    LOG(WARNING) << SlamLib::color::RED << "Relocalization() error: can't find local map,name: "
                        << name << SlamLib::color::RESET;
                    reloc_res.traj_id_ = -1;
                    return reloc_res; 
                }
                localmaps[name] = local_map; 
            }
            refine_registration_->SetInputSource(std::make_pair(name, localmaps[name]));  
        }
        refine_registration_->SetInputTarget(scan_in);
        if (!refine_registration_->Solve(res.second)) {
            reloc_res.traj_id_ = -1;
            return reloc_res; 
        }
        // step3 检验   
        std::string required_name = "filtered";    // 获取检验模块需要的点云标识名
        if (localmaps.find(required_name) == localmaps.end()) {
            PointCloudPtr local_map(new pcl::PointCloud<_PointType>());
            if (!ConstructLocalmapByTrajectoryNode(loop_vertex.traj_, search_ind, required_name, local_map)) {
                LOG(WARNING) << SlamLib::color::RED << "Relocalization() error: can't find local map,name: "
                    << required_name << SlamLib::color::RESET;
                reloc_res.traj_id_ = -1;
                return reloc_res; 
            }
            localmaps[required_name] = local_map; 
        }
        align_evaluator_.SetTargetPoints(localmaps[required_name]); 
        
        std::pair<double, double> eva = align_evaluator_.AlignmentScore(
            scan_in.at(required_name), res.second.matrix().cast<float>(), 0.2, 0.8); 
        tt.toc("Relocalization ");
        // score的物理意义 是 均方残差
        if (eva.first  > 0.1) {
            LOG(INFO) << "score: " << eva.first << SlamLib::color::RESET;
            // 对回环匹配失败的进行可视化检测
            #if (LOOP_DEBUG == 1)
                // 检测回环匹配是否准确  
                static uint16_t ind = 0; 
                typename pcl::PointCloud<_PointType>::Ptr res_points(
                    new pcl::PointCloud<_PointType>());
                pcl::PointCloud<_PointType> input_transformed;
                Eigen::Isometry3d rel_pose = res.second;
                // rel_pose.translation() = rel_pose.translation() + Eigen::Vector3d(10, 10, 10);
                // cloud 通过  relpose 转到  input_transformed  
                pcl::transformPointCloud (*scan_in.at("filtered"), input_transformed, rel_pose.matrix().cast<float>());
                *res_points = *localmaps["filtered"]; 
                *res_points += input_transformed;
                pcl::io::savePCDFileBinary("/home/lwh/reloc_res_" + std::to_string(ind++) + ".pcd", *res_points);
            #endif
            reloc_res.traj_id_ = -1; 
            return reloc_res;  
        }
        reloc_res.traj_id_ = loop_vertex.traj_;
        reloc_res.global_index_ = res.first;
        reloc_res.pose_ = res.second;  
        LOG(INFO) << SlamLib::color::GREEN << "relocalization success!";
        LOG(INFO) << "score: " << eva.first << SlamLib::color::RESET;
        return reloc_res;  
    }

    /**
     * @brief: 获取新增的回环信息
     */            
    std::deque<LoopEdge> GetNewLoops() {
        loop_mt_.lock(); 
        std::deque<LoopEdge> new_loop_copy = new_loops_;
        new_loops_.clear();
        loop_mt_.unlock();  
        return new_loop_copy;  
    }

    /**
     * @brief 保存回环模块数据    
     * 
     * @param path 
     */
    void Save(std::string const& path) {
        // std::cout << "loop Save: " << path << std::endl;
        scene_recognizer_.Save(path); 
    }

    /**
     * @brief 加载回环模块数据    
     * 
     * @param path 
     */
    void Load(std::string const& path, const uint64_t& max_index) {
        scene_recognizer_.Load(path, max_index);   // 场景识别模块加载数据
    }

    /**
     * @brief: 从轨迹id为curr_trajectory_id_的历史关键帧中查找位置与给定目标接近的
     * @details: 
     * @param max_search_dis 最远搜索距离  如果>0 那么有效，<=0则是不考虑搜索距离 
     * @return 查找到的数量 
     */            
    int HistoricalPositionSearch(const uint16_t& traj, pcl::PointXYZ const& pos, double const& max_search_dis,
                                                                uint16_t const& max_search_num, std::vector<int>& search_ind,
                                                                std::vector<float>& search_dis) {   
        // 如果目标搜索轨迹  与 当前模块的加载轨迹不一致  那么要重新加载轨迹数据库
        if (traj != curr_trajectory_id_) {
            // 变化则要更新位姿搜索kdtree 
            pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud =
                PoseGraphDataBase::GetInstance().GetKeyFramePositionCloud(traj);  
            kdtreeHistoryKeyPoses->setInputCloud(keyframe_position_cloud);
            last_keyframe_position_kdtree_size_ = keyframe_position_cloud->size();
            curr_trajectory_id_ = traj;  
        } else if (last_keyframe_position_kdtree_size_ != 
                PoseGraphDataBase::GetInstance().GetTrajectorVertexNum(curr_trajectory_id_)) {  
            pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud =
                PoseGraphDataBase::GetInstance().GetKeyFramePositionCloud(curr_trajectory_id_);  
            kdtreeHistoryKeyPoses->setInputCloud(keyframe_position_cloud);
            last_keyframe_position_kdtree_size_ = keyframe_position_cloud->size();
        }
        if (max_search_dis <= 0) {  // knn
            return kdtreeHistoryKeyPoses->nearestKSearch(pos, max_search_num, search_ind, search_dis); 
        }
        // 在历史关键帧中查找与当前关键帧距离小于阈值的集合  
        return kdtreeHistoryKeyPoses->radiusSearch(pos, max_search_dis, search_ind, search_dis, 
                                                                                        max_search_num); 
    }

    /**
     * @brief 用一条轨迹上的结点去构造一个Local map 
     * 
     * @param traj 轨迹的id 
     * @param search_ind 选择的轨迹结点的局部index  
     * @param points_name 选择的点的名字   
     * @param[out] local_map 输出目标local map   
     * @return true 
     * @return false 
     */
    bool ConstructLocalmapByTrajectoryNode(uint16_t traj, std::vector<int> const& search_ind,
                                                                                                std::string const& points_name, 
                                                                                                PointCloudPtr& local_map) {
        pcl::PointCloud<_PointType> origin_points;   // 激光坐标系下的点云
        pcl::PointCloud<_PointType> trans_points;   // 转换到世界坐标系下的点云 
        local_map->clear();  
        // 遍历每一个index的结点
        for (const int& idx : search_ind) {
            Vertex vertex = PoseGraphDataBase::GetInstance().GetVertexByTrajectoryLocalIndex(traj, idx);
            if (!PoseGraphDataBase::GetInstance().GetKeyFramePointCloud(points_name, vertex.id_, origin_points)) {
                return false;
            }
            pcl::transformPointCloud (origin_points, trans_points, vertex.pose_.matrix()); // 转到世界坐标  
            *local_map += trans_points; 
        }
        return true;  
    }

protected:
    /**
     * @brief 回环检测线程 
    */
    void LoopDetect() {
        while(1) {
            // 读取最新添加到数据库的关键帧进行回环检测  
            PoseGraphDataBase& poseGraph_database = PoseGraphDataBase::GetInstance(); 
            if (wait_processed_keyframe_.empty()) {
                std::chrono::milliseconds dura(100);
                std::this_thread::sleep_for(dura);
                continue;  
            }
            processed_queue_mt_.lock();
            LoopKeyframeInfo curr_keyframe_info = wait_processed_keyframe_.front();
            wait_processed_keyframe_.pop_front();  
            processed_queue_mt_.unlock();
            // 回环频率控制 
            if (curr_keyframe_info.id_ - last_detect_id_ > DETECT_FRAME_INTERVAL_) {   
                SlamLib::time::TicToc tt;  
                last_detect_id_ = curr_keyframe_info.id_;  
                // 机器人位置点云 
                static pcl::PointCloud<pcl::PointXYZ>::Ptr last_keyframe_position_cloud(nullptr);
                pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud =
                    poseGraph_database.GetKeyFramePositionCloud();  
                // if (last_keyframe_position_kdtree_size_ < 2 * MIN_LOOP_FRAME_INTERVAL_) {
                //     std::chrono::milliseconds dura(50);
                //     std::this_thread::sleep_for(dura);
                //     continue;  
                // }
                // // 1、基于位置进行搜索
                // // 在历史关键帧中查找与当前关键帧距离小于阈值的集合  
                // // tt.tic(); 
                // std::vector<int> search_ind_;  
                // std::vector<float> search_dis_;
                // kdtreeHistoryKeyPoses->radiusSearch(
                //     keyframe_position_cloud->points[curr_keyframe_.id_], 
                //     2 * MAX_LOOP_DISTANCE_, 
                //     search_ind_, 
                //     search_dis_, 
                //     0);    // 设置搜索的最大数量  如果是0表示不限制数量 
                // // tt.toc("kdtreeHistoryKeyPoses->radiusSearch  ");  // 非常小  乎略不计  

                // // 在候选关键帧集合中，找到与当前帧间隔较远的帧 作为候选帧  
                // int short_range_loop_id = -1;
                // float short_range_loop_dis = 1000;
                // int long_range_loop_id = -1;
                // float long_range_loop_dis = 1000;
                // // 从距离近的帧开始遍历
                // for (int i = 0; i < (int)search_ind_.size(); ++i) {
                //     int loop_interval = curr_keyframe_.id_ - search_ind_[i];
                //     // 闭环间隔小与1000个关键帧认为是小距离，此时认为里程计是基本准确的，x,y,z的误差不会太大
                //     // 而当闭环间隔大与1000个关键帧时认为是远距离闭环，此时里程计的信任度下降，全局描述子启用。
                //     if (loop_interval < 1000) {
                //         // odom的误差认为足够小 
                //         // 所以直接用kdtree搜索最近的帧作为闭环候选帧
                //         if (short_range_loop_id < 0) {
                //             short_range_loop_id = search_ind_[i];
                //             short_range_loop_dis = std::sqrt(search_dis_[i]);
                //             // 如果闭环的关键帧间隔足够大，且几何距离足够小 ，那么距离回环成立
                //             // std::cout << "近距离   loop_id: " << search_ind_[i] 
                //             //     << ", dis: " << short_range_loop_dis << std::endl;
                //         }
                //     } else if (loop_interval >= 1000) {
                //         // 远距离回环，选择kdtree搜索距离最近的帧作为回环候选帧
                //         if (long_range_loop_id < 0) {
                //             long_range_loop_id = search_ind_[i];
                //             long_range_loop_dis = std::sqrt(search_dis_[i]);
                //             // std::cout << "远距离   loop_id: " << search_ind_[i]
                //             //     << ", dis: " << long_range_loop_dis << std::endl;
                //         }
                //     }
                // }

                // if (short_range_loop_id == -1 && long_range_loop_id == -1) {
                //     // 大范围内不存在可回环历史帧，因此可长时间不进行回环检测
                //     DETECT_FRAME_INTERVAL_ = 30; 
                //     std::cout << "距离历史帧较远，降低回环检测频率, DETECT_FRAME_INTERVAL_ = 30" << std::endl;
                //     continue;   
                // } else {
                //     // 当前帧与历史帧最小距离超过60m时，认为短时间没有回环发生的可能，
                //     // 对于近距离历史帧，由与里程计误差较小，60m距离的可靠性很高。
                //     // 对于远距离历史帧，虽然里程计误差变大，但是不至于产生60m的误差，
                //     // 里程计超过60m的误差只可能在极大场景中发生，而极大场景会融合GPS...
                //     // 所以一旦当前帧与历史帧的距离超过60m，就认为短时间不会出现回环。
                //     if (long_range_loop_dis > 60 && short_range_loop_dis > 60) {
                //         DETECT_FRAME_INTERVAL_ = 20;       // 降低检测间隔，提高检测频率 
                //         std::chrono::milliseconds dura(50);
                //         std::this_thread::sleep_for(dura);
                //         continue;   // 回环不会出现，因此直接跳出
                //     } else if (long_range_loop_dis > 20 && short_range_loop_dis > 20) {
                //         DETECT_FRAME_INTERVAL_ = 8;    // 进一步提高频率 
                //     } else {
                //         DETECT_FRAME_INTERVAL_ = 3;     // 进一步提高频率 
                //     }

                //     std::cout << "距离历史帧较近，增加回环检测频率, DETECT_FRAME_INTERVAL_ ="
                //         << DETECT_FRAME_INTERVAL_ << std::endl;
                // }
                DETECT_FRAME_INTERVAL_ = 8; 
                std::pair<int64_t, Eigen::Isometry3d> res{-1, Eigen::Isometry3d::Identity()};
                // 近距离回环时，优先使用几何位置回环检测，因为近距离odom误差较小，位置检测->描述子检测
                // 远距离回环时，优先使用全局描述子回环检测，描述子检测->位置检测
                // if (long_range_loop_dis < short_range_loop_dis) {
                    // 远距离回环
                    std::cout << "远距离回环" << std::endl;
                    tt.tic(); 
                    // 场景识别模块工作  寻找相似帧
                    res = scene_recognizer_.LoopDetect(curr_keyframe_info.global_index_);   // 传入待识别的帧id 
                    tt.toc(" scene_recognize  ");  // 2ms
                    Vertex loop_vertex;  // 回环结点  
                    // 如果场景识别没有找到相似帧   用位置搜索继续找回环
                    if (res.first == -1) {
                        // std::cout << "远距离回环，描述子搜索失败，转为位置搜索，id: " << long_range_loop_id
                        //     << ",dis: " << long_range_loop_dis << std::res.first, endl;
                        // // 回环最远接收 x-y 10m的距离(太远匹配就不准，影响回环的准确性)，
                        // // 假设远距离回环时，odom的误差最大接受 x-y 20m, 那么远距离回环时odom的最大距离就是30m
                        // res.first = long_range_loop_id;  
                        // poseGraph_database.SearchVertexPose(curr_keyframe_.id_, res.second);  // 读取当前帧的pose 
                        // // 同一个地点  z轴的值应该相同
                        // Eigen::Isometry3d historical_pose;
                        // poseGraph_database.SearchVertexPose(res.first, historical_pose);  // 读取回环候选帧的pose 
                        // res.second.translation()[2] = historical_pose.translation()[2];   // 位姿初始值
                        // // 计算x-y 的距离
                        // float plane_dis = (res.second.translation() - historical_pose.translation()).norm();  
                        
                        // if (plane_dis > 30) {
                        //     std::chrono::milliseconds dura(50);
                        //     std::this_thread::sleep_for(dura);
                        //     continue;   // 回环不会出现，因此直接跳出
                        // }
                        // tt.tic(); 
                        continue;  
                    } else {
                        // 获取回环的结点信息
                        loop_vertex = poseGraph_database.GetVertexByDatabaseIndex(res.first);          
                        std::cout << "回环traj: " << loop_vertex.traj_ << std::endl;
                        // 当前帧位姿转换到世界系下
                        res.second = loop_vertex.pose_ * res.second;    // 位姿初始值
                    }
                    if (loop_vertex.traj_ != curr_trajectory_id_) {
                        std::cout << "跨轨迹回环! " << std::endl;
                        // 变化则要更新位姿搜索kdtree 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud =
                            PoseGraphDataBase::GetInstance().GetKeyFramePositionCloud(loop_vertex.traj_);  
                        kdtreeHistoryKeyPoses->setInputCloud(keyframe_position_cloud);
                        last_keyframe_position_kdtree_size_ = keyframe_position_cloud->size();
                        std::cout << "last_keyframe_position_kdtree_size_: " << last_keyframe_position_kdtree_size_ << std::endl;
                        curr_trajectory_id_ = loop_vertex.traj_;  
                    } else {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud =
                            PoseGraphDataBase::GetInstance().GetKeyFramePositionCloud(curr_trajectory_id_); 
                        // 判断是否需要更新位姿kdtree  
                        if (keyframe_position_cloud->size() - last_keyframe_position_kdtree_size_ 
                                >= MIN_LOOP_FRAME_INTERVAL_ ) {
                            if (last_keyframe_position_cloud == nullptr) {
                                last_keyframe_position_cloud = keyframe_position_cloud;
                                last_keyframe_position_kdtree_size_ = keyframe_position_cloud->size();  
                                continue; 
                            }
                            // SlamLib::time::TicToc tt;
                            std::cout << SlamLib::color::GREEN << "updata loop kdtree! size: " 
                                << last_keyframe_position_cloud->size() << SlamLib::color::RESET << std::endl;
                            kdtreeHistoryKeyPoses->setInputCloud(last_keyframe_position_cloud);
                            // tt.toc("updata loop kdtree ");    // 耗时 0.5 ms 
                            last_keyframe_position_cloud = keyframe_position_cloud;
                            last_keyframe_position_kdtree_size_ = keyframe_position_cloud->size();  
                        } 
                        std::cout << "同轨迹回环! " << std::endl;
                        std::cout << "last_keyframe_position_kdtree_size: " << last_keyframe_position_kdtree_size_
                            << ", 当前size: " << keyframe_position_cloud->size() << std::endl;
                    }
                    // 通过kdtree 在同一条轨迹的历史结点中搜索若干个距离回环结点最接近的若干结点
                    pcl::PointXYZ loop_vertex_position; 
                    loop_vertex_position.x = loop_vertex.pose_.translation().x();
                    loop_vertex_position.y = loop_vertex.pose_.translation().y();
                    loop_vertex_position.z = loop_vertex.pose_.translation().z();
                    std::vector<int> search_ind;
                    std::vector<float> search_dis;
                    if(kdtreeHistoryKeyPoses->radiusSearch(loop_vertex_position, 30, search_ind, search_dis, 10) == 0) {
                        LOG(INFO) << "回环目标 kdtree 近邻搜索的数量为0";
                        continue;
                    }
                // } 
                // else {
                //     std::cout << "近距离回环，位置搜索，id: " << short_range_loop_id
                //             << ",dis: " << short_range_loop_dis << std::endl;
                //     // 回环最远接收 x-y 10m的距离(太远匹配就不准，影响回环的准确性)，
                //     // 假设近距离回环时，odom的误差最大接受 x-y 10m, 那么远距离回环时odom的最大距离就是20m
                //     res.first = short_range_loop_id;  
                //     poseGraph_database.SearchVertexPose(curr_keyframe_.id_, res.second);  // 读取当前帧的pose 
                //     // 同一个地点  z轴的值应该相同
                //     Eigen::Isometry3d historical_pose;
                //     poseGraph_database.SearchVertexPose(res.first, historical_pose);  // 读取回环候选帧的pose 
                //     res.second.translation()[2] = historical_pose.translation()[2];   // 位姿初始值
                //     // 计算x-y 的距离
                //     float plane_dis = (res.second.translation() - historical_pose.translation()).norm();  
                    
                //     if (plane_dis > 20) {
                //         std::chrono::milliseconds dura(50);
                //         std::this_thread::sleep_for(dura);
                //         continue;   // 回环不会出现，因此直接跳出
                //     }
                // }
                // Eigen::Isometry3d origin_T = res.second;  
                // 粗匹配
                std::unordered_map<std::string, PointCloudConstPtr> localmaps;  
                FeaturePointcloudContainer  curr_scans; 
                // 将粗匹配所需要的点云提取出来 
                // 这里允许 匹配使用多种特征点云  
                for (auto const& name : rough_registration_specific_labels_) {   
                    // local map 
                    typename pcl::PointCloud<_PointType>::Ptr local_map(new pcl::PointCloud<_PointType>());
                    // std::cout << "粗匹配提取点云,name: " << name << std::endl;
                    if (!ConstructLocalmapByTrajectoryNode(loop_vertex.traj_, search_ind, name, local_map)) {
                        continue;  
                    }
                    rough_registration_->SetInputSource(std::make_pair(name, local_map)); 
                    localmaps[name] = local_map; 
                    // 当前点云
                    typename pcl::PointCloud<_PointType>::Ptr curr_scan(new pcl::PointCloud<_PointType>());
                    if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(name, 
                            curr_keyframe_info.id_, curr_scan)) {
                        std::cout << SlamLib::color::RED << "loop rough, Find curr points error "
                            << name << SlamLib::color::RESET << std::endl;
                        continue;
                    }
                    curr_scans[name] = curr_scan; 
                }
                rough_registration_->SetInputTarget(curr_scans);
                // 回环first的点云
                if (!rough_registration_->Solve(res.second)) {
                    continue; 
                }
                /**
                 * @todo 简单判断粗匹配的质量
                 */
                // 将细匹配所需要的点云提取出来 
                for (auto const& name : refine_registration_specific_labels_) {
                    // 如果 细匹配所需要的local map 在之前粗匹配时  已经提取了   那么直接用原数据
                    if (localmaps.find(name) == localmaps.end()) {
                        typename pcl::PointCloud<_PointType>::Ptr local_map(new pcl::PointCloud<_PointType>());
                        if (!ConstructLocalmapByTrajectoryNode(loop_vertex.traj_, search_ind, name, local_map)) {
                            continue;  
                        }
                        localmaps[name] = local_map; 
                    }
                    refine_registration_->SetInputSource(std::make_pair(name, localmaps[name]));  
                    // 如果 细匹配所需要的 curr_data 不存在  则构造
                    if (curr_scans.find(name) == curr_scans.end()) {
                        typename pcl::PointCloud<_PointType>::Ptr curr_scan(new pcl::PointCloud<_PointType>());
                        if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(name, 
                                curr_keyframe_info.id_, curr_scan)) {
                            std::cout << SlamLib::color::RED << "loop refine, find curr points error "
                                << name << SlamLib::color::RESET << std::endl;
                            continue;
                        }
                        curr_scans[name] = curr_scan; 
                    }
                }
                refine_registration_->SetInputTarget(curr_scans);
                if (!refine_registration_->Solve(res.second)) {
                    continue; 
                }
                // 对细匹配进行评估
                // 使用标识名为evaluative_pointcloud_label_的点云进行评估  
                if (localmaps.find(evaluative_pointcloud_label_) != localmaps.end()) {
                    align_evaluator_.SetTargetPoints(localmaps[evaluative_pointcloud_label_]); 
                } else {  
                    typename pcl::PointCloud<_PointType>::Ptr evaluative_local_map(new pcl::PointCloud<_PointType>());
                    if (!ConstructLocalmapByTrajectoryNode(loop_vertex.traj_, search_ind, evaluative_pointcloud_label_, evaluative_local_map)) {
                        continue;  
                    }
                    align_evaluator_.SetTargetPoints(evaluative_local_map); 
                }
                
                typename pcl::PointCloud<_PointType>::ConstPtr evaluative_curr_scan(
                    new pcl::PointCloud<_PointType>());
                // 用于检验的当前点云
                if (curr_scans.find(evaluative_pointcloud_label_) != curr_scans.end()) {
                    evaluative_curr_scan = curr_scans[evaluative_pointcloud_label_];
                } else {
                    typename pcl::PointCloud<_PointType>::Ptr evaluative_curr_scan_temp(new pcl::PointCloud<_PointType>());
                    if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(evaluative_pointcloud_label_, 
                            curr_keyframe_info.id_, evaluative_curr_scan_temp)) {
                        std::cout << SlamLib::color::RED << "Find evaluative curr points error, name: "
                            << evaluative_pointcloud_label_ << SlamLib::color::RESET << std::endl;
                        continue;  
                    }
                    evaluative_curr_scan = evaluative_curr_scan_temp;  
                }
                
                std::pair<double, double> eva = align_evaluator_.AlignmentScore(
                    evaluative_curr_scan, 
                    res.second.matrix().cast<float>(), 
                    SCORE_THRESH_, 
                    OVERLAP_THRESH_); 
                std::cout << SlamLib::color::GREEN << "loop refine match converged, score: " 
                    << eva.first << std::endl;
                Eigen::Isometry3d origin_T = res.second;  

                if (eva.first > MIN_SCORE_) {
                    std::cout << "eva.first > MIN_SCORE_, eva.first: " << eva.first
                        << ",MIN_SCORE_: " << MIN_SCORE_ << std::endl;
                    // 对回环匹配失败的进行可视化检测
                    #if (LOOP_DEBUG == 1)
                        // 检测回环匹配是否准确  
                        static uint16_t ind = 0; 
                        typename pcl::PointCloud<_PointType>::Ptr res_points(
                            new pcl::PointCloud<_PointType>());
                        pcl::PointCloud<_PointType> input_transformed;
                        // cloud 通过  relpose 转到  input_transformed  
                        pcl::transformPointCloud (*curr_scans["filtered"], input_transformed, origin_T.matrix());
                        *res_points = *localmaps["filtered"]; 
                        *res_points += input_transformed;
                        pcl::io::savePCDFileBinary("/home/lwh/loop_res_" + std::to_string(ind++) + ".pcd", *res_points);
                    #endif
                    continue;  
                }
                // 添加新增回环边
                LoopEdge new_loop; 
                new_loop.loop_traj_ = loop_vertex.traj_;  
                new_loop.link_id_.first = curr_keyframe_info.id_;
                new_loop.link_id_.second = loop_vertex.id_;
                new_loop.link_head_local_index_ = curr_keyframe_info.local_index_;   
                new_loop.constraint_ = res.second.inverse() * loop_vertex.pose_;
                new_loop.loop_vertex_pose_ = loop_vertex.pose_;
                std::cout << "loop link_id_.first: " << new_loop.link_id_.first
                    << ", link_id_.second: " << loop_vertex.id_ << std::endl;
                // Eigen::Matrix<double, 1, 6> noise;
                // noise << 0.0025, 0.0025, 0.0025, 0.0001, 0.0001, 0.0001;
                /**
                 * @todo 回环协方差应该要根据匹配质量动态设置 
                 */
                new_loop.noise_ << 0.0025, 0.0025, 0.0025, 0.0001, 0.0001, 0.0001;
                loop_mt_.lock();
                new_loops_.push_back(std::move(new_loop));  
                loop_mt_.unlock();  
            }
            std::chrono::milliseconds dura(50);
            std::this_thread::sleep_for(dura);
        }
    }
private:    
    double DETECT_TIME_INTERVAL_ = 1.0;   // 检测的最小时间间隔    s   
    uint16_t DETECT_FRAME_INTERVAL_ = 3;   // 检测的最小帧间隔     
    uint16_t MIN_LOOP_FRAME_INTERVAL_ = 50;    //  一个回环的最小关键帧间隔  
    double MAX_LOOP_DISTANCE_ = 50;   //  回环之间   的最大距离   单位m 
    double MAX_ODOM_ERROR_ = 50;  // 最大里程计误差  
    double SCORE_THRESH_ = 0;
    double OVERLAP_THRESH_ = 0;
    double MIN_SCORE_ = 0;

    std::mutex processed_queue_mt_;  
    std::mutex loop_mt_; 
    KeyFrame::Ptr curr_keyframe_; 
    double last_detect_time_ = 0;    
    int32_t curr_trajectory_id_ = -1;   
    uint32_t last_detect_id_ = 0;  
    uint32_t last_keyframe_position_kdtree_size_ = 0;  
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses;  // 历史关键帧的位姿kdtree 
    std::thread loop_thread_;  

    RegistrationPtr rough_registration_;
    RegistrationPtr refine_registration_;

    std::deque<LoopKeyframeInfo> wait_processed_keyframe_;  
    std::deque<LoopEdge> new_loops_;  
    // 针对 _PointType 点云的 匹配质量评估器 
    SlamLib::pointcloud::PointCloudAlignmentEvaluate<_PointType> align_evaluator_;
    SceneRecognitionScanContext<_PointType> scene_recognizer_; 

    std::vector<std::string> rough_registration_specific_labels_;
    std::vector<std::string> refine_registration_specific_labels_;
    std::string evaluative_pointcloud_label_;  
}; // class 
} // namespace 
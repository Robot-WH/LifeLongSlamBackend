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
 * @brief: 基于雷达的闭环检测 - 1、先验位姿检测   2、点云描述子检测 
 *@param _PointType 回环时进行点云匹配的点类型 
    */    
template<typename _PointType>
class LoopDetection {   
private:
    using PointCloudConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr;  
    using SourceT = std::pair<std::string, PointCloudConstPtr>;     // 匹配源类型    <id, 数据>
    using FeatureContainer = SlamLib::FeaturePointCloudContainer<_PointType>;
    using RegistrationPtr = std::unique_ptr<SlamLib::pointcloud::RegistrationBase<_PointType>>;   
public:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

        // for (auto name : rough_registration_specific_labels_) {
        //     std::cout << "闭环粗匹配name: " << name << std::endl;
        // }

        evaluative_pointcloud_label_ = "filtered";
    }

    virtual ~LoopDetection() {}

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 传入一帧激光数据 
     * @details 放入处理队列中 
     * @param {*}
     * @return {*}
     */            
    void AddData(FeatureContainer const& scan_in) {
        scene_recognizer_.AddKeyFramePoints(scan_in);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 基于激光点云的重定位 
     * @details: 
     * @param scan_in 输入全部激光信息
     * @return <匹配历史帧的id, 重定位位姿>
     */            
    std::pair<int64_t, Eigen::Isometry3d> Relocalization(
            FeatureContainer const& scan_in) {   
        SlamLib::time::TicToc tt; 
        // step1 先识别出相似帧  
        std::pair<int64_t, Eigen::Isometry3d> res = scene_recognizer_.FindSimilarPointCloud(scan_in);  
        if (res.first == -1) return res; 
        // step2 采用粗匹配 + 细匹配模式求解出位姿
        PoseGraphDataBase& poseGraph_database = PoseGraphDataBase::GetInstance(); 
        std::unordered_map<std::string, typename pcl::PointCloud<_PointType>::ConstPtr> owned_localmap;  
        // 将粗匹配所需要的点云local map 提取出来 
        for (auto const& name : rough_registration_specific_labels_) {   
            // 从数据库中查找 名字为 name 的点云 
            typename pcl::PointCloud<_PointType>::ConstPtr local_map(new pcl::PointCloud<_PointType>());
            if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(res.first, 5, name, local_map)) {
                LOG(WARNING) << SlamLib::color::RED<<"ERROR: find local map !"<< name <<SlamLib::color::RESET;
                res.first = -1;
                return res;  
            }
            rough_registration_->SetInputSource(std::make_pair(name, local_map)); 
            owned_localmap[name] = local_map; 
        }
        rough_registration_->SetInputTarget(scan_in);
        // 当前帧位姿转换到世界系下
        Eigen::Isometry3d historical_pose;
        if (!poseGraph_database.SearchVertexPose(res.first, historical_pose)) {
            LOG(WARNING) << SlamLib::color::RED << "ERROR: not find historical pose "
                << SlamLib::color::RESET;
            res.first = -1;
            return res;  
        }
        res.second = historical_pose * res.second;  
        // 回环first的点云
        if (!rough_registration_->Solve(res.second)) {
            res.first = -1;
            return res;  
        }
        // 细匹配
        // 将细匹配所需要的点云local map 提取出来 
        for (auto const& name : refine_registration_specific_labels_) {
            typename pcl::PointCloud<_PointType>::ConstPtr local_map(new pcl::PointCloud<_PointType>());
            // 如果 细匹配所需要的local map 在之前粗匹配时  已经提取了
            if (owned_localmap.find(name) != owned_localmap.end()) {
                local_map = owned_localmap[name];  
            } else {
                if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                        res.first, 5, name, local_map)) {
                    res.first = -1;
                    return res;  
                }
                owned_localmap[name] = local_map; 
            }
            refine_registration_->SetInputSource(std::make_pair(name, local_map));  
        }
        refine_registration_->SetInputTarget(scan_in);
        if (!refine_registration_->Solve(res.second)) {
            res.first = -1;
            return res;  
        }
        // step3 检验   
        typename pcl::PointCloud<_PointType>::ConstPtr local_map(new pcl::PointCloud<_PointType>());
        std::string required_name = "processed";    // 获取检验模块需要的点云标识名
        if (owned_localmap.find(required_name) != owned_localmap.end()) {
            local_map = owned_localmap[required_name];  
        } else {  // 如果之前没有构造出 POINTS_PROCESSED_NAME 的local map 那么这里构造
            if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                    res.first, 5, required_name, local_map)) {
                res.first = -1;
                return res;  
            }
        }
        align_evaluator_.SetTargetPoints(local_map); 
        std::pair<double, double> eva = align_evaluator_.AlignmentScore(scan_in.at(required_name), 
                                                                                                                res.second.matrix().cast<float>(), 0.1, 0.6); 
        tt.toc("Relocalization ");
        // score的物理意义 是 均方残差
        if (eva.first > 0.05) {
            res.first = -1;
            return res;  
        }
        LOG(INFO) << SlamLib::color::GREEN << "relocalization success!";
        LOG(INFO) << "score: " << eva.first << SlamLib::color::RESET;
        return res;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 获取新增的回环信息
     */            
    std::deque<Edge> GetNewLoops() {
        loop_mt_.lock(); 
        std::deque<Edge> new_loop_copy = new_loops_;
        new_loops_.clear();
        loop_mt_.unlock();  
        return new_loop_copy;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 保存回环模块数据    
    void Save(std::string const& path) {
        scene_recognizer_.Save(path); 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 加载回环模块数据    
    void Load(std::string const& path) {
        scene_recognizer_.Load(path);   // 场景识别模块加载数据
        pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud;
        keyframe_position_cloud = PoseGraphDataBase::GetInstance().GetKeyFramePositionCloud();
        
        if (!keyframe_position_cloud->empty()) {
            kdtreeHistoryKeyPoses->setInputCloud(keyframe_position_cloud);
            LOG(INFO) << SlamLib::color::GREEN << "位置点云KDTREE初始化完成! 数量：" << 
                keyframe_position_cloud->size() << SlamLib::color::RESET;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 在历史关键帧中查找位置与给定目标接近的
     * @details: 
     * @param max_search_dis 最远搜索距离  如果>0 那么有效，<=0则是不考虑搜索距离 
     * @return 查找到的数量 
     */            
    int HistoricalPositionSearch(pcl::PointXYZ const& pos, double const& max_search_dis,
                                                                uint16_t const& max_search_num, std::vector<int> &search_ind,
                                                                std::vector<float> &search_dis) {   
        // 检查是否需要更新位置点云   确保包含历史所有节点的位置信息
        if (kdtree_size_ != PoseGraphDataBase::GetInstance().ReadVertexNum()) {  
            pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud =
                PoseGraphDataBase::GetInstance().GetKeyFramePositionCloud();  
            kdtreeHistoryKeyPoses->setInputCloud(keyframe_position_cloud);
            kdtree_size_ = keyframe_position_cloud->size();
        }

        if (max_search_dis <= 0) {
            // KNN搜索
            return kdtreeHistoryKeyPoses->nearestKSearch(pos, max_search_num, search_ind, search_dis); 
        }
        // 在历史关键帧中查找与当前关键帧距离小于阈值的集合  
        return kdtreeHistoryKeyPoses->radiusSearch(pos, max_search_dis, search_ind, search_dis, 
                                                                                        max_search_num); 
    }

protected:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 回环检测线程 
    */
    void LoopDetect() {
        while(1) {
            // 读取最新添加到数据库的关键帧进行回环检测  
            KeyFrame curr_keyframe_;
            PoseGraphDataBase& poseGraph_database = PoseGraphDataBase::GetInstance(); 

            if (!poseGraph_database.GetNewKeyFrame(curr_keyframe_)) {
                std::chrono::milliseconds dura(50);
                std::this_thread::sleep_for(dura);
                continue;  
            }
            // 每隔1s 进行一次回环检测，且本次回环检测至少与上一个回环相距3个帧  避免回环过于密集
            if (curr_keyframe_.time_stamp_ - last_detect_time_ > DETECT_TIME_INTERVAL_
                    && curr_keyframe_.id_ - last_detect_id_ > DETECT_FRAME_INTERVAL_) {   
                SlamLib::time::TicToc tt;  
                last_detect_time_ = curr_keyframe_.time_stamp_;
                last_detect_id_ = curr_keyframe_.id_;  
                // 机器人位置点云 
                pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe_position_cloud =
                    poseGraph_database.GetKeyFramePositionCloud();  

                if (keyframe_position_cloud->size() < MIN_LOOP_FRAME_INTERVAL_) {
                    std::chrono::milliseconds dura(50);
                    std::this_thread::sleep_for(dura);
                    continue;  
                }
                // 判断是否需要更新位姿kdtree  
                if (keyframe_position_cloud->size() - kdtree_size_ 
                        >= MIN_LOOP_FRAME_INTERVAL_ ) {
                    // SlamLib::time::TicToc tt;
                    std::cout << SlamLib::color::GREEN << "updata loop kdtree!" << 
                        SlamLib::color::RESET << std::endl;
                    kdtreeHistoryKeyPoses->setInputCloud(keyframe_position_cloud);
                    kdtree_size_ = keyframe_position_cloud->size();  
                    // tt.toc("updata loop kdtree ");    // 耗时 0.5 ms 
                }
                // tt.tic(); 
                // 场景识别模块工作  寻找相似帧
                std::pair<int64_t, Eigen::Isometry3d> res{-1, Eigen::Isometry3d::Identity()};
                res = scene_recognizer_.LoopDetect(curr_keyframe_.id_);   // 传入待识别的帧id 
                // tt.toc(" scene_recognize  ");  // 2ms
                // 如果场景识别没有找到相似帧   用位置搜索继续找回环
                if (res.first == -1) {
                    // 在历史关键帧中查找与当前关键帧距离小于阈值的集合  
                    std::vector<int> search_ind_;  
                    std::vector<float> search_dis_;
                    kdtreeHistoryKeyPoses->radiusSearch(
                        keyframe_position_cloud->points[curr_keyframe_.id_], 
                        MAX_LOOP_DISTANCE_, 
                        search_ind_, 
                        search_dis_, 
                        0);    // 设置搜索的最大数量  如果是0表示不限制数量 
                    // 在候选关键帧集合中，找到与当前帧间隔较远的帧 作为候选帧  
                    // 从距离近的帧开始遍历
                    for (int i = 0; i < (int)search_ind_.size(); ++i) {
                        int id = search_ind_[i];
                        // 当前帧的idx 减去 搜索到的帧的idx  足够 大 ，表示间隔足够远
                        if ((keyframe_position_cloud->size() - 1) - id > MIN_LOOP_FRAME_INTERVAL_) {
                            std::cout << SlamLib::color::GREEN << "位姿搜索回环成功!, id: " << id 
                                << ", curr id: " << curr_keyframe_.id_ << SlamLib::color::RESET << std::endl;
                            res.first = id;
                            break;
                        }
                    }

                    // if (res.first == -1) {
                        std::chrono::milliseconds dura(50);
                        std::this_thread::sleep_for(dura);
                        continue;  
                    // }
                    // // poseGraph_database.SearchKeyFramePose(curr_keyframe_.id_, res.second);  // 读取该帧的pose 
                    // poseGraph_database.SearchVertexPose(curr_keyframe_.id_, res.second);  // 读取该帧的pose 
                    // // 同一个地点  z轴的值应该相同
                    // Eigen::Isometry3d historical_pose;
                    // // poseGraph_database.SearchKeyFramePose(res.first, historical_pose);
                    // poseGraph_database.SearchVertexPose(res.first, historical_pose);  // 读取该帧的pose 
                    // res.second.translation()[2] = historical_pose.translation()[2];   // 位姿初始值
                    // tt.tic(); 
                } else {
                    // 当前帧位姿转换到世界系下
                    Eigen::Isometry3d historical_pose;
                    poseGraph_database.SearchVertexPose(res.first, historical_pose);
                    res.second = historical_pose * res.second;    // 位姿初始值
                }
                // 粗匹配
                std::unordered_map<std::string, typename pcl::PointCloud<_PointType>::ConstPtr> localmaps;  
                FeatureContainer  curr_scans; 
                // 将粗匹配所需要的点云提取出来 
                // 这里允许 匹配使用多种特征点云  
                for (auto const& name : rough_registration_specific_labels_) {   
                    // local map 
                    typename pcl::PointCloud<_PointType>::ConstPtr local_map(new pcl::PointCloud<_PointType>());
                    
                    if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                                res.first, 
                                5,  // 前后5个帧组成local map  
                                name, 
                                local_map)
                        ) {
                        std::cout << SlamLib::color::RED << "find local map error " << name 
                            << SlamLib::color::RESET << std::endl;    
                        continue;
                    }

                    rough_registration_->SetInputSource(std::make_pair(name, local_map)); 
                    localmaps[name] = local_map; 
                    // 当前点云
                    typename pcl::PointCloud<_PointType>::Ptr curr_scan(new pcl::PointCloud<_PointType>());
                    
                    if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(name, 
                            curr_keyframe_.id_, curr_scan)) {
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
                    typename pcl::PointCloud<_PointType>::ConstPtr local_map(new pcl::PointCloud<_PointType>());
                    // 如果 细匹配所需要的local map 在之前粗匹配时  已经提取了   那么直接用原数据
                    if (localmaps.find(name) != localmaps.end()) {
                        local_map = localmaps[name];  
                    } else {
                        if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                                res.first, 5, name, local_map)) {
                            continue;  
                        }

                        localmaps[name] = local_map; 
                    }

                    refine_registration_->SetInputSource(std::make_pair(name, local_map));  
                    // 如果 细匹配所需要的 curr_data 不存在  则构造
                    if (curr_scans.find(name) == curr_scans.end()) {
                        typename pcl::PointCloud<_PointType>::Ptr curr_scan(new pcl::PointCloud<_PointType>());

                        if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(name, 
                                curr_keyframe_.id_, curr_scan)) {
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
                typename pcl::PointCloud<_PointType>::ConstPtr evaluative_local_map(new pcl::PointCloud<_PointType>());
                // 使用标识名为evaluative_pointcloud_label_的点云进行评估  
                if (localmaps.find(evaluative_pointcloud_label_) != localmaps.end()) {
                    evaluative_local_map = localmaps[evaluative_pointcloud_label_];  
                } else {  
                    if (!poseGraph_database.GetAdjacentLinkNodeLocalMap<_PointType>(
                            res.first, 5, evaluative_pointcloud_label_, evaluative_local_map)) {
                        continue; 
                    }
                }

                align_evaluator_.SetTargetPoints(evaluative_local_map); 

                typename pcl::PointCloud<_PointType>::ConstPtr evaluative_curr_scan(
                    new pcl::PointCloud<_PointType>());
                // 用于检验的当前点云
                if (curr_scans.find(evaluative_pointcloud_label_) != curr_scans.end()) {
                    evaluative_curr_scan = curr_scans[evaluative_pointcloud_label_];
                } else {
                    typename pcl::PointCloud<_PointType>::Ptr evaluative_curr_scan_temp(new pcl::PointCloud<_PointType>());
                    
                    if (!poseGraph_database.GetKeyFramePointCloud<_PointType>(evaluative_pointcloud_label_, 
                            curr_keyframe_.id_, evaluative_curr_scan_temp)) {
                        std::cout << SlamLib::color::RED << "Find evaluative curr points ERROR, name: "
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
                
                if (eva.first > MIN_SCORE_) {
                    continue;  
                }

                #if (LOOP_DEBUG == 1)
                    // 检测回环匹配是否准确  
                    static uint16_t ind = 0; 
                    typename pcl::PointCloud<_PointType>::Ptr res_points(new pcl::PointCloud<_PointType>());
                    *res_points = *local_map + *aligned; 
                    pcl::io::savePCDFileBinary("/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loop_res_"
                                                                            + std::to_string(ind++) + ".pcd"
                                                                                , *res_points);
                #endif
                // 添加新增回环边
                Edge new_loop; 
                new_loop.link_id_.first = res.first;
                new_loop.link_id_.second = curr_keyframe_.id_;
                Eigen::Isometry3d historical_pose;
                poseGraph_database.SearchVertexPose(res.first, historical_pose);
                new_loop.constraint_ = historical_pose.inverse() * res.second;// T second -> first
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
    uint16_t MIN_LOOP_FRAME_INTERVAL_ = 100;    //  一个回环的最小间隔  
    double MAX_LOOP_DISTANCE_ = 10;   //  回环之间   的最大距离  
    double MAX_ODOM_ERROR_ = 50;  // 最大里程计误差  
    double SCORE_THRESH_ = 0;
    double OVERLAP_THRESH_ = 0;
    double MIN_SCORE_ = 0;


    std::mutex pcl_mt_;  
    std::mutex loop_mt_; 
    std::deque<FeatureContainer> pointcloud_process_; 
    KeyFrame::Ptr curr_keyframe_; 
    double last_detect_time_ = -100;    
    uint32_t last_detect_id_ = 0;  
    uint32_t kdtree_size_ = 0;  
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses;  // 历史关键帧的位姿kdtree 
    std::thread loop_thread_;  

    RegistrationPtr rough_registration_;
    RegistrationPtr refine_registration_;

    std::deque<Edge> new_loops_;  
    // 针对 _PointType 点云的 匹配质量评估器 
    SlamLib::pointcloud::PointCloudAlignmentEvaluate<_PointType> align_evaluator_;
    SceneRecognitionScanContext<_PointType> scene_recognizer_; 

    std::vector<std::string> rough_registration_specific_labels_;
    std::vector<std::string> refine_registration_specific_labels_;
    std::string evaluative_pointcloud_label_;  
}; // class 
} // namespace 


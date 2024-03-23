/**
 * @file backend_lifelong_impl.hpp
 * @author lwh ()
 * @brief 
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 */
#pragma once 
#include "backend_lifelong.h"
#include "SlamLib/PointCloud/Filter/voxel_grid.h"
#include "InnerComm/InnerProcessComm.hpp"
#include <yaml-cpp/yaml.h>
namespace lifelong_backend {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _FeatureT>
LifeLongBackEndOptimization<_FeatureT>::LifeLongBackEndOptimization(std::string config_path) {
    YAML::Node yaml = YAML::LoadFile(config_path);
    std::string optimizer_type = yaml["optimizer"]["type"].as<std::string>();

    if (optimizer_type == "gtsam") {
        optimizer_ = std::make_unique<GtsamGraphOptimizer>(); 
    } else if (optimizer_type == "g2o") {
        optimizer_ = std::make_unique<G2oGraphOptimizer>(); 
    } else if (optimizer_type == "ceres") {
    }
    // 将定位匹配算法默认设置为NDT 
    SlamLib::pointcloud::ndtomp_ptr<_FeatureT> ndt = 
        SlamLib::pointcloud::CreateNDTOMP<_FeatureT>(
            1.0,   // ndt  cell size 
            0.01,   // 
            0.1, 
            30,  // 最大迭代次数  
            4,   // 线程数量
            "DIRECT7"
        );
    localize_registration_.reset(new SlamLib::pointcloud::PCLRegistration<_FeatureT>(
        std::move(ndt), "filtered"));  
    // 构造回环检测
    LoopDetectionOption loop_detection_option;
    loop_detection_option.min_score = yaml["loop_detect"]["evaluate"]["min_score"].as<float>();
    loop_detection_option.overlap_thresh = yaml["loop_detect"]["evaluate"]["overlap_thresh"].as<float>();
    loop_detection_option.score_thresh = yaml["loop_detect"]["evaluate"]["score_thresh"].as<float>();
    std::cout << "闭环检测参数，min_score：" << loop_detection_option.min_score
        << " ,overlap_thresh: " << loop_detection_option.overlap_thresh
        << " ,score_thresh: " << loop_detection_option.score_thresh << std::endl;
    loop_detect_ = std::make_unique<LoopDetection<_FeatureT>>(loop_detection_option); 
    work_mode_ = WorkMode::SLEEP;  
    mapping_thread_ = std::thread(&LifeLongBackEndOptimization::mapping, this);  // 启动线程  
    localization_thread_ = std::thread(&LifeLongBackEndOptimization::localization, this);   
    new_external_constranit_num_ = 0; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 
 * 
 * @tparam _FeatureT 
 * @param space_path 
 * @return std::vector<uint16_t> 轨迹id 的 list  
 */
template<typename _FeatureT>
std::vector<uint16_t> LifeLongBackEndOptimization<_FeatureT>::Load(std::string space_path) {  
    option_.database_save_path_ = space_path;  
    std::vector<uint16_t> traj_id_list;  
    work_mode_ = WorkMode::MAPPING;  
    // pose graph 加载
    if (!PoseGraphDataBase::GetInstance().Load(space_path)) {
        LOG(INFO) << SlamLib::color::YELLOW << "数据库为空，准备建图...... " << SlamLib::color::RESET;
        trajectory_ = PoseGraphDataBase::GetInstance().CreateNewSession(); 
        loop_detect_->Load(space_path, 0);
        return traj_id_list;
    }  
    loop_detect_->Load(space_path, PoseGraphDataBase::GetInstance().GetDataBaseInfo().keyframe_cnt); // 加载场景识别数据库 
    traj_id_list = lifelong_backend::PoseGraphDataBase::GetInstance().GetTrajectoryIDList();
    work_mode_ = WorkMode::RELOCALIZATION;   // 默认为建图模式 
    LOG(INFO) << SlamLib::color::GREEN << "载入历史数据库，准备重定位......" << SlamLib::color::RESET;
    trajectory_ = traj_id_list.front();      // 默认使用轨迹0
    return std::move(traj_id_list);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _FeatureT>
void LifeLongBackEndOptimization<_FeatureT>::SaveGlobalMap(float resolution, std::string save_path) {
        typename pcl::PointCloud<_FeatureT>::Ptr global_map(new pcl::PointCloud<_FeatureT>());
    // 遍历全部节点
    uint64_t num = PoseGraphDataBase::GetInstance().GetGraphVertexNum();
    for (uint64_t i = 0; i < num; i++) {
        typename pcl::PointCloud<_FeatureT>::Ptr origin_points(new pcl::PointCloud<_FeatureT>());
        pcl::PointCloud<_FeatureT> trans_points;   // 转换到世界坐标系下的点云 
        // 读取该节点的点云
        PoseGraphDataBase::GetInstance().GetKeyFramePointCloud<_FeatureT>(
            "filtered", i, origin_points);
        // 读取节点的位姿
        Eigen::Isometry3d pose;
        PoseGraphDataBase::GetInstance().SearchVertexPose(i, pose);
        pcl::transformPointCloud(*origin_points, trans_points, pose.matrix()); // 转到世界坐标  
        *global_map += trans_points;
    }
    // 对global_map进行滤波
    SlamLib::pointcloud::FilterOption::VoxelGridFilterOption option;
    option.mode_ = "VoxelGrid";
    option.voxel_grid_option_.resolution_ = resolution;
    SlamLib::pointcloud::VoxelGridFilter<_FeatureT> voxel_filter(option);
    typename pcl::PointCloud<_FeatureT>::Ptr cloud_out = voxel_filter.Filter(global_map); 
    pcl::io::savePCDFileBinary(save_path + "/global_map.pcd", *cloud_out);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _FeatureT>
void LifeLongBackEndOptimization<_FeatureT>::SavePoseGraph() {
    ForceGlobalOptimaze();  
    PoseGraphDataBase::GetInstance().Save();    // 数据库保存
    std::cout << "数据库保存成功！" << std::endl;
    loop_detect_->Save(option_.database_save_path_);    // 回环检测数据库保存 
    std::cout << "场景识别数据保存成功！" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _FeatureT>
bool LifeLongBackEndOptimization<_FeatureT>::SetTrajectory(uint16_t traj_id) {
    std::cout << "SetTrajectory ,traj_id: " << traj_id << std::endl;
    if (!PoseGraphDataBase::GetInstance().FindTrajectory(traj_id)) {
        return false;
    }
    trajectory_ = traj_id;
    // 可视化历史轨迹
    KeyFrameInfo<_FeatureT> keyframe_info; 
    keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetTrajectoryVertex(trajectory_); 
    keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetTrajectoryEdge(trajectory_); 
    IPC::Server::Instance().Publish("keyframes_info", keyframe_info); 
    std::cout << "vertex_database_ size: " << keyframe_info.vertex_database_.size() << std::endl;
    return true;  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _FeatureT>
uint8_t LifeLongBackEndOptimization<_FeatureT>::SetWorkMode(uint16_t cmd) {
    if (cmd == 1) {
        // 设置为lifelong  
        if (enable_lifelong_) {
            return 0;
        }
        enable_lifelong_ = true;
    } else if (cmd == 2) {
        // 设置为纯定位
        if (work_mode_ == WorkMode::LOCALIZATION && !enable_lifelong_) {
            return 0;  
        } else {
            if (work_mode_ == WorkMode::SLEEP || work_mode_ == WorkMode::MAPPING) {
                work_mode_ = WorkMode::LOCALIZATION;
            }
            if (enable_lifelong_) {
                enable_lifelong_ = false; 
            }
        }
    } else if (cmd == 3) {
        // 设置为纯建图
        if (work_mode_ == WorkMode::MAPPING && !enable_lifelong_) {
            return 0;  
        } else {
            if (work_mode_ != WorkMode::MAPPING) {
                work_mode_ = WorkMode::MAPPING;
            }
            if (enable_lifelong_) {
                enable_lifelong_ = false; 
            }
            // 如果当前轨迹有数据，那么重新开始建立一条独立的轨迹
            if (PoseGraphDataBase::GetInstance().GetTrajectorVertexNum(trajectory_)) {
                trajectory_ = PoseGraphDataBase::GetInstance().CreateNewSession(); 
            }
        }
    }
    return 1; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
template<typename _FeatureT>
void LifeLongBackEndOptimization<_FeatureT>::AddKeyFrame(
                    SlamLib::CloudContainer<_FeatureT> const& lidar_data, 
                    Eigen::Isometry3d const& odom) {
    static double last_keyframe_t = -1;
    // 检查时间戳 看是否乱序    
    if (lidar_data.timestamp_start_ <= last_keyframe_t) {
        LOG(ERROR) << SlamLib::color::RED << "Backend -- AddKeyFrame(): timestamp error"
            << SlamLib::color::RESET;
        return;
    }
    if (last_keyframe_t < 0) {
        last_keyframe_odom_ = odom;  
    }
    last_keyframe_t = lidar_data.timestamp_start_; 

    if (work_mode_ == WorkMode::SLEEP) {
        return;  
    }
    if (work_mode_ == WorkMode::RELOCALIZATION) {
        // 重定位
        LOG(INFO) << SlamLib::color::GREEN << "-----------------RELOCALIZATION!-----------------" 
            << SlamLib::color::RESET;
        RelocResult reloc_res = loop_detect_->Relocalization(lidar_data.pointcloud_data_); 
        static uint8_t n = 0;  
        // 重定位失败 ，若开启地图更新，此时会重新建立一条新的轨迹，否则延迟一下，继续重定位 
        if (reloc_res.traj_id_ < 0) {
            if (enable_lifelong_) {
                // 连续几帧重定位失败则建立新地图 
                if (n > 10) {
                    std::cout << SlamLib::color::GREEN << "重定位失败，新建轨迹..." 
                        << SlamLib::color::RESET << std::endl;
                    n = 0;
                    work_mode_ = WorkMode::MAPPING;
                    optimizer_->Reset();  
                    trajectory_ = PoseGraphDataBase::GetInstance().CreateNewSession(); 
                }
                n++;
            }
        } else {
            n = 0;
            work_mode_ = WorkMode::LOCALIZATION; // 进入定位模式 
            this->trans_odom2map_ = reloc_res.pose_ * odom.inverse(); 
            IPC::Server::Instance().Publish("odom_to_map", this->trans_odom2map_);      // 发布坐标变换
            trajectory_ = reloc_res.traj_id_;
            optimizer_->Rebuild(PoseGraphDataBase::GetInstance().GetTrajectoryVertex(trajectory_),
                                                        PoseGraphDataBase::GetInstance().GetTrajectoryEdge(trajectory_));  
            // 可视化历史轨迹
            KeyFrameInfo<_FeatureT> keyframe_info; 
            keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetTrajectoryVertex(trajectory_); 
            keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetTrajectoryEdge(trajectory_); 
            IPC::Server::Instance().Publish("keyframes_info", keyframe_info); 
        }
        return; 
    }
    std::unique_lock<std::shared_mutex> lock(this->keyframe_queue_sm_);
    // 将输入的数据放入待处理队列 
    KeyFrame keyframe(lidar_data.timestamp_start_, odom);
    // 获取该关键帧与上一关键帧之间的相对约束  last <- curr
    keyframe.between_constraint_ = odom.inverse() * last_keyframe_odom_;  
    last_keyframe_odom_ = odom;    
    this->new_keyframe_queue_.push_back(keyframe);     
    this->new_keyframe_points_queue_.push_back(lidar_data.pointcloud_data_); 
    // 如果是建图阶段   则需要发布图优化相关数据   供其他模块使用    
    if (work_mode_ != WorkMode::MAPPING) {
        return;  
    }
    KeyFrameInfo<_FeatureT> keyframe_info; 
    keyframe_info.time_stamps_ = keyframe.time_stamp_;  
    keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetTrajectoryVertex(trajectory_); 
    keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetTrajectoryEdge(trajectory_); 
    keyframe_info.new_keyframes_ = this->new_keyframe_queue_;  
    IPC::Server::Instance().Publish("keyframes_info", keyframe_info);   // 发布图关键帧  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 强制执行一次全局优化   save的时候用
template<typename _FeatureT>
void LifeLongBackEndOptimization<_FeatureT>::ForceGlobalOptimaze() {
    optimizer_->Optimize();  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _FeatureT>
void LifeLongBackEndOptimization<_FeatureT>::localization() {
    while(1) {
        if (work_mode_ != WorkMode::LOCALIZATION) {
            std::chrono::milliseconds dura(100);
            std::this_thread::sleep_for(dura);
            continue;  
        }
        this->keyframe_queue_sm_.lock_shared();
        // assert(this->new_keyframe_queue_.size() == this->new_keyframe_points_queue_.size()); 
        // 如果没有新的关键帧  
        if (this->new_keyframe_queue_.empty()) {
            this->keyframe_queue_sm_.unlock_shared();
            std::chrono::milliseconds dura(10);
            std::this_thread::sleep_for(dura);
            continue;  
        }
        // static double avg_t = 0;
        // static int n = 1;  
        // 降低定位频率  
        static int interval = 1;
        if (interval == 0) {
            interval = 1;  
            std::cout << SlamLib::color::GREEN << "-----------------LOCALIZATION!-----------------" << std::endl;
            // 取出最早的帧  进行map匹配
            /**
             * @todo 这里读取deque时被读锁上锁了，有没有必要呢？了解deque在写数据时，引用有不有可能会失效  
             */
            KeyFrame& keyframe = this->new_keyframe_queue_.front(); 
            SlamLib::FeaturePointCloudContainer<_FeatureT>& points = this->new_keyframe_points_queue_.front();
            this->keyframe_queue_sm_.unlock_shared();    // 读锁  解锁
            Eigen::Isometry3d pose_in_map = this->trans_odom2map_ * keyframe.odom_;  
            pcl::PointXYZ curr_pos(pose_in_map.translation().x(), 
                                                            pose_in_map.translation().y(), 
                                                            pose_in_map.translation().z());
            std::vector<int> search_ind;
            std::vector<float> search_dis;   // 这个是距离的平方 !! 
            SlamLib::time::TicToc tt;  
            loop_detect_->HistoricalPositionSearch(trajectory_, curr_pos, 20, 10, search_ind, search_dis);   // 搜索最近的历史关键帧
            // tt.toc("HistoricalPositionSearch ");
            // for (auto& i : search_ind) {
            //     std::cout << "knn id:" << 
            //         PoseGraphDataBase::GetInstance().GetVertexByTrajectoryLocalIndex(trajectory_, i).id_ 
            //  << std::endl;
            // }
            /**
             *如果啥都搜不到呢？   如果啥都搜不到说明当前位姿不确定了需要重定位
             */                    
            if (!search_ind.empty()) {
                LocalizationPointsInfo<_FeatureT> loc_points;  
                tt.tic();
                // 给局部地图进行降采样的滤波器
                SlamLib::pointcloud::FilterOption::VoxelGridFilterOption filter_option;
                filter_option.mode_ = "VoxelGrid";
                filter_option.voxel_grid_option_.resolution_ = 0.3;  
                SlamLib::pointcloud::VoxelGridFilter<_FeatureT> downsample(filter_option);
                /**
                 * @todo 目前是直接提取一定范围的关键帧组合为定位地图，之后要改为，每个区域维护一个submap，
                 *  直接提取这个区域的submap进行定位，这个submap也具备更新的能力 
                 */
                // 遍历定位所需的所有点云标识名   将定位所需要的点云local map 提取出来 
                for (auto const& name : localize_registration_->GetUsedPointsName()) {   
                    // 从数据库中查找 名字为 name 的点云 
                    PointCloudConstPtr local_map(new pcl::PointCloud<_FeatureT>());
                    if (!loop_detect_->ConstructLocalmapByTrajectoryNode(trajectory_, search_ind, name, local_map)) {
                        work_mode_ = WorkMode::RELOCALIZATION;
                        continue; 
                    }
                    // std::cout << "before filter size: " << local_map->size() << std::endl;
                    local_map = downsample.Filter(local_map); 
                    // std::cout << "after filter size: " << local_map->size() << std::endl;
                    localize_registration_->SetInputSource(std::make_pair(name, local_map)); 
                    loc_points.map_[name] = local_map; 
                    // SlamLib::time::TicToc tt;  
                    IPC::Server::Instance().Publish("localize_map", local_map);   // 发布地图用于可视化    
                    // tt.toc("Publish ");   // 1ms
                }
                tt.toc("build map ");
                loc_points.time_stamps_ = keyframe.time_stamp_; 
                localize_registration_->SetInputTarget(points);
                tt.tic(); 
                if (!localize_registration_->Solve(pose_in_map)) {
                    LOG(WARNING)<<SlamLib::color::RED<<"错误: 定位匹配无法收敛！转换到重定位模式..."
                        <<SlamLib::color::RESET;
                    work_mode_ = WorkMode::RELOCALIZATION;
                    continue;  
                }
                //std::cout<<SlamLib::color::GREEN<<"after loc pose_in_map: "<<pose_in_map.matrix()<<std::endl;
                double t = tt.toc("localization ");
                // avg_t += (t - avg_t) / n;
                // n++;  
                // std::cout << "avg_t: " << avg_t << std::endl;
                // 更为全面的评估定位的状态，环境的变化
                std::cout << "评估 " << std::endl;
                tt.tic(); 
                // 匹配评估
                typename pcl::PointCloud<_FeatureT>::ConstPtr evaluate_local_map(
                    new pcl::PointCloud<_FeatureT>());
                std::string required_name = "filtered";    // 获取检验模块需要的点云标识名
                // 检查定位匹配阶段 是否已经构建了定位评估阶段所需要使用的地图 
                if (loc_points.map_.find(required_name) != loc_points.map_.end()) {
                    evaluate_local_map = loc_points.map_[required_name];  
                } else {  
                    // 没有所需要的地图数据则进行构建
                    typename pcl::PointCloud<_FeatureT>::ConstPtr local_map(new pcl::PointCloud<_FeatureT>());
                    if (!loop_detect_->ConstructLocalmapByTrajectoryNode(trajectory_, search_ind, required_name, local_map)) {
                        work_mode_ = WorkMode::RELOCALIZATION;
                        continue; 
                    }
                    evaluate_local_map = local_map;
                }
                align_evaluator_.SetTargetPoints(evaluate_local_map);      // 0-15ms 
                std::pair<double, double> res = align_evaluator_.AlignmentScore(points.at(required_name), 
                                                                                    pose_in_map.matrix().cast<float>(), 0.2, 0.3); // 0-10ms 
                tt.toc("evaluate ");                                                                                                          
                std::cout<<"score: "<<res.first<<std::endl;
                std::cout<<"overlap_ratio: "<<res.second<<std::endl;
                static double last_loc_record_time = -1;
                static float last_loc_record_overlap = 0;  
                // 得分大于1, 认为定位失败 
                if (res.first > 1)  {  
                    // 进行重定位
                    work_mode_ = WorkMode::RELOCALIZATION;
                    // #if (debug == 1)
                    //     pcl::PointCloud<_FeatureT> input_transformed;
                    //     // cloud 通过  relpose 转到  input_transformed  
                    //     pcl::transformPointCloud (*points.at(required_name), 
                    //                                                             input_transformed, 
                    //                                                             pose_in_map.matrix().cast<float>());
                    //     static uint16_t ind = 0; 
                    //     typename pcl::PointCloud<_FeatureT>::Ptr res_points(new pcl::PointCloud<_FeatureT>());
                    //     *res_points = *evaluate_local_map;
                    //         *res_points += input_transformed; 
                    //     pcl::io::savePCDFileBinary(
                    //         "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_all.pcd"
                    //         , *res_points);
                    //     pcl::io::savePCDFileBinary(
                    //         "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_map.pcd"
                    //         , *evaluate_local_map);
                    //     pcl::io::savePCDFileBinary(
                    //         "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_scan.pcd"
                    //         , *points.at(required_name));
                    //     pcl::io::savePCDFileBinary(
                    //         "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_scan_trans.pcd"
                    //         , input_transformed);
                    // #endif
                    continue;  
                }
                if (enable_lifelong_) {
                    // 如果得分很低 ， 认为定位很好，同时若重叠率也足够小，则认为环境发生了较大的变化   
                    if (res.first <= 0.15 && res.second < 0.7) {
                        double min_historical_keyframe_dis = std::sqrt(search_dis.front()); 
                        // 相比于历史轨迹距离足够远就直接进行建图
                        if (min_historical_keyframe_dis > 10) {
                            // 转为纯建图模式
                            // 搜索最近历史结点的位姿 
                            Eigen::Isometry3d front_pose; 
                            // PoseGraphDataBase::GetInstance().SearchVertexPose(search_ind[0], front_pose); 
                            Vertex nearest_vertex = PoseGraphDataBase::GetInstance().GetVertexByTrajectoryLocalIndex(trajectory_, search_ind[0]);
                            Eigen::Isometry3d relpose = pose_in_map.inverse() * nearest_vertex.pose_;    // 与最近历史结点的想对位姿
                            std::cout << "扩展建图，连接的轨迹：" << trajectory_ << ", 连接的结点id: " << nearest_vertex.id_
                                << ", relpose: " << std::endl << relpose.matrix() << std::endl;
                            // 重新设置该关键帧的连接关系  与 约束 
                            keyframe.adjacent_id_ = nearest_vertex.id_;  
                            keyframe.between_constraint_ = relpose;      // 获取该关键帧与上一关键帧之间的相对约束  last<-curr       
                            work_mode_ = WorkMode::MAPPING;
                            this->trans_odom2map_ = pose_in_map * keyframe.odom_.inverse();  
                            IPC::Server::Instance().Publish("odom_to_map", this->trans_odom2map_);      // 发布坐标变换
                            continue; 
                        } else {
                            // 转为地图更新模式  
                            std::cout << SlamLib::color::GREEN << "-----------------MAP UDAPATE!-----------------" 
                                << SlamLib::color::RESET << std::endl;
                            /**
                             * @todo 阶段一：只更新定位地图 ，但是位姿图不更新
                             *                  阶段二：更新定位地图，位姿图以及回环数据库也同步更新 
                             */
                        }
                    } 
                    // 记录重叠率  
                    if (keyframe.time_stamp_ - last_loc_record_time > 1) {
                        last_loc_record_time = keyframe.time_stamp_;
                        last_loc_record_overlap = res.second ;  
                    }
                    // 发生下面情况则切换到建图模式 
                    // 1、如果重叠率降低到阈值一下，且降低缓慢
                    // 2、匹配得分较高
                    // 3、最近历史关键帧的距离过远 > 10 m 
                }
                // 更新校正矩阵  
                this->trans_odom2map_ = pose_in_map * keyframe.odom_.inverse();  
                IPC::Server::Instance().Publish("odom_to_map", this->trans_odom2map_);      // 发布坐标变换
            } else {
                work_mode_ = WorkMode::RELOCALIZATION;
            }
        } else {
            this->keyframe_queue_sm_.unlock_shared();
            --interval;
        }
        this->keyframe_queue_sm_.lock();
        this->new_keyframe_queue_.pop_front(); 
        this->new_keyframe_points_queue_.pop_front();
        this->keyframe_queue_sm_.unlock(); 
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 后端建图线程  
 */         
template<typename _FeatureT>   
void LifeLongBackEndOptimization<_FeatureT>::mapping() {
    while(true) {
        if (work_mode_ != WorkMode::MAPPING) {
            std::chrono::milliseconds dura(1000);
            std::this_thread::sleep_for(dura);
            continue;  
        }
        // 数据处理 
        if (!processData()) {
            std::chrono::milliseconds dura(1000);
            std::this_thread::sleep_for(dura);
            continue;  
        }
        PoseGraphDataBase& database = PoseGraphDataBase::GetInstance();  
        if (optimize()) {   
            // std::cout << "optimize ok" << std::endl;
            // SlamLib::time::TicToc tt;
            // 优化完成后 更新数据库  
            std::vector<Vertex>& curr_traj_vertex = database.GetTrajectoryVertex(trajectory_);
            for (auto& vertex : curr_traj_vertex) {
                vertex.pose_ = optimizer_->GetNodePose(vertex.id_); 
            }
            this->trans_odom2map_ = optimizer_->GetNodePose(last_add_database_keyframe_.id_) 
                                                                    * last_add_database_keyframe_.odom_.inverse();  
            IPC::Server::Instance().Publish("odom_to_map", this->trans_odom2map_);      // 发布坐标变换
            if (enable_lifelong_ && has_loop_) {
                // 进入定位模式前需要更新一下可视化 
                KeyFrameInfo<_FeatureT> keyframe_info; 
                keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetTrajectoryVertex(trajectory_); 
                keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetTrajectoryEdge(trajectory_); 
                keyframe_info.new_keyframes_ = this->new_keyframe_queue_;  
                IPC::Server::Instance().Publish("keyframes_info", keyframe_info);   // 发布图关键帧  
                work_mode_ = WorkMode::LOCALIZATION; // 进入定位模式 
                // std::cout << "建图线程：进入定位模式！" << std::endl;
                // localization_thread_ = std::thread(&LifeLongBackEndOptimization::localization, this);  
                has_loop_ = false;  
            }
        }
        std::chrono::milliseconds dura(1000);
        std::this_thread::sleep_for(dura);
    }
}    

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 对于新加入的关键帧new_keyframe_queue_，对每一个关键帧匹配约束，
 *  约束对应好后放入 wait_optimize_keyframes_容器
 */            
template<typename _FeatureT>   
bool LifeLongBackEndOptimization<_FeatureT>::processData() {
    std::unique_lock<std::shared_mutex> lock(this->keyframe_queue_sm_);
    if (this->new_keyframe_queue_.empty()) {
        return false;
    }
    // std::cout << "trajectory: " << trajectory_ << ", size: " << 
    // PoseGraphDataBase::GetInstance().GetTrajectoryVertex(trajectory_).size() << std::endl;
    // 处理的数量  
    int num_processed = std::min<int>(this->new_keyframe_queue_.size(), 
                                                                                max_keyframes_per_update_);
    last_add_database_keyframe_ = this->new_keyframe_queue_[num_processed - 1];
    // 遍历全部关键帧队列       
    for (int i = 0; i < num_processed; i++) {
        // 从keyframe_queue中取出关键帧
        KeyFrame& keyframe = this->new_keyframe_queue_[i];
        FeaturePointCloudContainer& points = this->new_keyframe_points_queue_[i];  
        Eigen::Isometry3d corrected_pose = this->trans_odom2map_ * keyframe.odom_; 
        //  添加到数据库中       返回添加到数据库中的 id 和 local_index  
        std::pair<uint32_t, uint32_t> id_localIndex = 
            PoseGraphDataBase::GetInstance().AddKeyFrame<_FeatureT>(trajectory_, corrected_pose, points);   
        if (i == num_processed - 1) {
            last_add_database_keyframe_.id_ = id_localIndex.first;  
        }
        // 关键帧数据加入到回环模块请求回环检测    
        loop_detect_->AddRequest(id_localIndex.first, id_localIndex.second, points);    
        // 每段轨迹的第一个结点 fix 
        if (id_localIndex.second == 0) {
            optimizer_->AddSe3Node(corrected_pose, id_localIndex.first, true);
            continue;
        }
        optimizer_->AddSe3Node(corrected_pose, id_localIndex.first); 
        // std::cout << "AddSe3Node, id: " << id_localIndex.first << std::endl;
        // 观测噪声
        Eigen::Matrix<double, 1, 6> noise;
        noise << 0.0025, 0.0025, 0.0025, 0.0001, 0.0001, 0.0001;
        // 如果邻接节点id 没有被设置  说明就是与上一个节点连接 
        if (keyframe.adjacent_id_ == -1) {
            keyframe.adjacent_id_ = id_localIndex.first - 1;  
        }
        optimizer_->AddSe3Edge(id_localIndex.first, keyframe.adjacent_id_, keyframe.between_constraint_, noise);  
        PoseGraphDataBase::GetInstance().AddEdge(trajectory_, id_localIndex.first, keyframe.adjacent_id_, 
                                                                                                        id_localIndex.second , keyframe.between_constraint_, noise);  
        // std::cout << "AddEdge, trajectory_: " << trajectory_ << " adjacent_id: " << keyframe.adjacent_id_ << ", curr_id: " << id_localIndex.first
        //     << std::endl;
        // 与GNSS进行匹配 
        // 寻找有无匹配的GPS   有则
        // if (!pairGnssOdomInSingle(GNSS_queue, keyframe))                // return true 匹配完成   false 继续等待数据  
        // { 
        //     if (ros::Time::now().toSec() - keyframe->stamp.toSec() < 1)   // 最多等待1s 
        //     {
        //         break;
        //     }
        // }
        // else
        // { 
        //     // 匹配上GNSS数据了
        //     // 按照一定频率添加GNSS约束  
        //     static int gnss_freq_count = 0;
        //     if(gnss_freq_count <= 0) 
        //     {
        //         keyframe->GNSS_Valid = true;  
        //         gnss_freq_count = GNSS_optimize_freq;  
        //     }
        //     else 
        //     {
        //         gnss_freq_count--;
        //     }
            
        //     if(keyframe->gnss_matched) 
        //     {
        //         Eigen::Matrix4d gt_pose = Eigen::Matrix4d::Identity();  
                
        //         gt_pose.block<3,1>(0, 3) = keyframe->utm_coord; 
        //         gt_pose.block<3,3>(0, 0) =  keyframe->orientation.matrix(); 
        //         SaveTrajectory(gt_pose, keyframe->Pose.matrix(), "/slam_data/trajectory", "/slam_data/trajectory/ground_truth.txt",
        //                         "/slam_data/trajectory/est_path.txt");
        //     }
        // }
        // 按照一定频率添加先验平面约束  
        if (enable_planeConstraint_optimize_) {
            static int planeConstraint_freq_count = 0;
            if (planeConstraint_freq_count <= 0) {
                keyframe.planeConstraint_valid_ = true;  
                planeConstraint_freq_count = planeConstraint_optimize_freq_;  
            } else {
                planeConstraint_freq_count--;
            }
        }     
    }
    this->new_keyframe_queue_.erase(this->new_keyframe_queue_.begin(), 
        this->new_keyframe_queue_.begin() + num_processed);     //  [first,last) 
    this->new_keyframe_points_queue_.erase(this->new_keyframe_points_queue_.begin(), 
        this->new_keyframe_points_queue_.begin() + num_processed);     //  [first,last) 
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 全局pose graph优化
 * @details: 执行全局pose-graph 优化的条件  1、有回环发生    2、有足够多的新增外部约束 
 * @return 是否进行了优化 
 */      
template<typename _FeatureT>         
bool LifeLongBackEndOptimization<_FeatureT>::optimize() {
    static bool do_optimize = false;  
    PoseGraphDataBase& database = PoseGraphDataBase::GetInstance();
    std::deque<LoopEdge>  new_loops = loop_detect_->GetNewLoops();
    // std::cout << "optimize(), new_loops size: " << new_loops.size() << std::endl;
    if (new_loops.size() > 0) {
        has_loop_ = true;  
        // 添加回环边
        for (uint16_t i = 0; i < new_loops.size(); i++) {
            // 与不同的轨迹发生了回环，较晚的轨迹与较早的轨迹对齐 
            if (new_loops[i].loop_traj_ != trajectory_) {
                // std::cout << "跨轨迹回环，merge..." << std::endl;
                // 提取出回环trajectory的全部节点  
                std::vector<Vertex>& loop_trajectory_vertexs = database.GetTrajectoryVertex(new_loops[i].loop_traj_);  
                Eigen::Isometry3d correct = Eigen::Isometry3d::Identity();
                Vertex curr_vertex = database.GetVertexByTrajectoryLocalIndex(trajectory_, new_loops[i].link_head_local_index_);  
                // std::cout << "loop_trajectory_vertexs size: " << loop_trajectory_vertexs.size() << std::endl;
                // 较早的轨迹作为基准，即与较早的估计进行对齐  
                if (new_loops[i].loop_traj_ < trajectory_) {
                    // 当前建图的轨迹与历史回环轨迹对齐(当前轨迹的节点转换到回环轨迹的参考坐标系上)
                    // std::cout << "curr_vertex, id: " << curr_vertex.id_ << std::endl;
                    correct = (new_loops[i].loop_vertex_pose_ * new_loops[i].constraint_.inverse()) * curr_vertex.pose_.inverse();
                    // 需要对当前轨迹的数据转换到回环轨迹的坐标系上 
                    auto& curr_traj_vertex = PoseGraphDataBase::GetInstance().GetTrajectoryVertex(trajectory_);
                    // std::cout << "curr_traj_vertex size: " << curr_traj_vertex.size() << std::endl;
                    // 重新设置当前轨迹的位姿和轨迹编号 
                    for (auto& vertex : curr_traj_vertex) {
                        vertex.pose_ = correct * vertex.pose_;  
                        vertex.traj_ = new_loops[i].loop_traj_;
                        optimizer_->SetNodePose(vertex.id_, vertex.pose_); 
                    }
                } else {
                    // 回环的轨迹与当前轨迹对齐
                    correct = (curr_vertex.pose_ * new_loops[i].constraint_) * new_loops[i].loop_vertex_pose_.inverse();
                }
                // 将回环的vertex 添加
                for (uint32_t n = 0; n < loop_trajectory_vertexs.size(); n++) {
                    if (new_loops[i].loop_traj_ > trajectory_) {
                        // 回环轨迹转换到当前轨迹坐标系上 
                        loop_trajectory_vertexs[n].pose_ = correct * loop_trajectory_vertexs[n].pose_;
                        loop_trajectory_vertexs[n].traj_ = trajectory_;  
                        // 更新位姿点云
                        PoseGraphDataBase::GetInstance().UpdataKeyframePointcloud(new_loops[i].loop_traj_, n, loop_trajectory_vertexs[n].pose_);
                    } 
                    optimizer_->AddSe3Node(loop_trajectory_vertexs[n].pose_, loop_trajectory_vertexs[n].id_); 
                }
                // 从数据库磁盘中加载对应轨迹的边  
                const std::vector<Edge>& loop_trajectory_edges = 
                    PoseGraphDataBase::GetInstance().GetTrajectoryEdge(new_loops[i].loop_traj_);
                for (auto& edge : loop_trajectory_edges) {
                    optimizer_->AddSe3Edge(edge.link_id_.first, edge.link_id_.second, 
                                                        edge.constraint_, edge.noise_);  
                }
                // 合并
                if (new_loops[i].loop_traj_ < trajectory_) {
                    PoseGraphDataBase::GetInstance().MergeTrajectory(new_loops[i].loop_traj_, trajectory_); 
                    trajectory_ = new_loops[i].loop_traj_;
                } else {
                    PoseGraphDataBase::GetInstance().MergeTrajectory(trajectory_, new_loops[i].loop_traj_); 
                }
            } else {
            //     std::cout << "new_loops[i].loop_traj_: " << new_loops[i].loop_traj_ 
            //             << ",trajectory_: " << trajectory_ << std::endl;
            // }
            }
            // std::cout << "loop head: " << new_loops[i].link_id_.first << ", tail: " << new_loops[i].link_id_.second << std::endl;
            optimizer_->AddSe3Edge(new_loops[i].link_id_.first, new_loops[i].link_id_.second, 
                                                                    new_loops[i].constraint_, new_loops[i].noise_);  
            // 回环数据记录到数据库中
            PoseGraphDataBase::GetInstance().AddEdge(trajectory_, new_loops[i]);  
        }
    }
    // 如果累计的外部约束超过10个  也触发一次优化 GetTrajectorVertexNum
    // 如GNSS 约束  + 地面约束 
    if (new_external_constranit_num_ > 10) {
        do_optimize = true; 
        new_external_constranit_num_ = 0; 
    }
    // optimize the pose graph
    // 如果新增足够的外部约束以及没有回环  那么不用执行全局优化 
    if (!do_optimize && !has_loop_) {
        return false;
    }  
    // 执行优化
    SlamLib::time::TicToc tt;
    optimizer_->Optimize(has_loop_);  
    tt.toc("optimizer_->Optimize ");
    do_optimize = false;  
    return true;  
}
}
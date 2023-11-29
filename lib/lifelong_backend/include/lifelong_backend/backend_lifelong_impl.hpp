/**
 * @file backend_lifelong_impl.hpp
 * @author lwh ()
 * @brief 
 * @version 0.1
 * @date 2023-11-25
 * 
 * @copyright Copyright (c) 2023
 * 
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
    
    work_mode_ = WorkMode::MAPPING;  
    mapping_thread_ = std::thread(&LifeLongBackEndOptimization::mapping, this);  // 启动线程  
    localization_thread_ = std::thread(&LifeLongBackEndOptimization::localization, this);   
    // 设置数据库保存路径
    option_.database_save_path_ = yaml["database_path"].as<std::string>();
    PoseGraphDataBase::GetInstance().SetSavePath(option_.database_save_path_);  
    Load();  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _FeatureT>
void LifeLongBackEndOptimization<_FeatureT>::Load() {    
    loop_detect_->Load(option_.database_save_path_); // 加载场景识别数据库 
    // pose graph 加载
    if (!PoseGraphDataBase::GetInstance().Load()) {
        LOG(INFO) << SlamLib::color::YELLOW << "数据库载入失败，准备建图...... " << SlamLib::color::RESET;
        return;
    }  

    work_mode_ = WorkMode::RELOCALIZATION;   // 默认为建图模式 
    LOG(INFO) << SlamLib::color::GREEN << "载入历史数据库，准备重定位......" << SlamLib::color::RESET;
    // 恢复历史图优化
    optimizer_->Rebuild(PoseGraphDataBase::GetInstance().GetAllVertex(),  // 所有的顶点
                                                PoseGraphDataBase::GetInstance().GetAllEdge());    // 所有的边                   
    // 从优化器中读回节点位姿
    for(int i=0; i < optimizer_->GetNodeNum(); i++) {
        PoseGraphDataBase::GetInstance().UpdateVertexPose(i, optimizer_->ReadOptimizedPose(i)); 
    }
    // 发布在载后的数据 
    KeyFrameInfo<_FeatureT> keyframe_info; 
    keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetAllVertex(); 
    keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetAllEdge(); 
    IPC::Server::Instance().Publish("keyframes_info", keyframe_info); 
    LOG(INFO) << SlamLib::color::GREEN << "历史轨迹可视化发布..." << SlamLib::color::RESET;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _FeatureT>
void LifeLongBackEndOptimization<_FeatureT>::SaveGlobalMap(float resolution, std::string save_path) {
        typename pcl::PointCloud<_FeatureT>::Ptr global_map(new pcl::PointCloud<_FeatureT>());
    // 遍历全部节点
    uint64_t num = PoseGraphDataBase::GetInstance().ReadVertexNum();

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
    loop_detect_->Save(option_.database_save_path_);    // 回环检测数据库保存 
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

    if (work_mode_ == WorkMode::RELOCALIZATION) {
        // 重定位
        LOG(INFO) << SlamLib::color::GREEN << "-----------------RELOCALIZATION!-----------------" 
            << SlamLib::color::RESET;
        std::pair<int64_t, Eigen::Isometry3d> res = loop_detect_->Relocalization(lidar_data.pointcloud_data_); 
        // 重定位失败 ，若开启地图更新，此时会重新建立一条新的轨迹，否则延迟一下，继续重定位 
        if (res.first == -1) {
            if (enable_map_update_) {
            } 
        } else {
            work_mode_ = WorkMode::LOCALIZATION; // 进入定位模式 
            this->trans_odom2map_ = res.second * odom.inverse(); 
            IPC::Server::Instance().Publish("odom_to_map", this->trans_odom2map_);      // 发布坐标变换
        }
        return; 
    }
    
    Eigen::Isometry3d between_constraint = last_keyframe_odom_.inverse() * odom;
    last_keyframe_odom_ = odom;  
    std::unique_lock<std::shared_mutex> lock(this->keyframe_queue_sm_);
    // 将输入的数据放入待处理队列 
    KeyFrame keyframe(lidar_data.timestamp_start_, odom);
    keyframe.between_constraint_ = between_constraint;      // 获取该关键帧与上一关键帧之间的相对约束  last<-curr 
    this->new_keyframe_queue_.push_back(keyframe);     
    this->new_keyframe_points_queue_.push_back(lidar_data.pointcloud_data_); 
    // 如果是建图阶段   则需要发布图优化相关数据   供其他模块使用    
    if (work_mode_ != WorkMode::MAPPING) {
        return;  
    }

    KeyFrameInfo<_FeatureT> keyframe_info; 
    keyframe_info.time_stamps_ = keyframe.time_stamp_;  
    keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetAllVertex(); 
    keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetAllEdge(); 
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
        // if (work_mode_ != LOCALIZATION) return;  
        if (work_mode_ != WorkMode::LOCALIZATION) {
            std::chrono::milliseconds dura(100);
            std::this_thread::sleep_for(dura);
            continue;  
        }

        this->keyframe_queue_sm_.lock_shared();
        assert(this->new_keyframe_queue_.size() == this->new_keyframe_points_queue_.size()); 
        // 如果没有新的关键帧  
        if (this->new_keyframe_queue_.empty()) {
            this->keyframe_queue_sm_.unlock_shared();
            std::chrono::milliseconds dura(10);
            std::this_thread::sleep_for(dura);
            continue;  
        }

        std::cout << SlamLib::color::GREEN << "-----------------LOCALIZATION!-----------------" << std::endl;
        // 取出最早的帧  进行map匹配
        KeyFrame& keyframe = this->new_keyframe_queue_.front(); 
        SlamLib::FeaturePointCloudContainer<_FeatureT>& points = this->new_keyframe_points_queue_.front();
        this->keyframe_queue_sm_.unlock_shared();
        SlamLib::time::TicToc tt;  
        Eigen::Isometry3d pose_in_map = this->trans_odom2map_ * keyframe.odom_;  
        pcl::PointXYZ curr_pos(pose_in_map.translation().x(), 
                                                        pose_in_map.translation().y(), 
                                                        pose_in_map.translation().z());
        std::vector<int> search_ind;
        std::vector<float> search_dis;   // 这个是距离的平方 !! 
        loop_detect_->HistoricalPositionSearch(curr_pos, 0, 10, search_ind, search_dis);   // 搜索最近的历史关键帧
        // std::cout<<"near node: "<<search_ind[0]<<std::endl;
        // std::cout << "search_dis.front: " << std::sqrt(search_dis.front()) << ", back: " << std::sqrt(search_dis.back()) << std::endl;
        /**
         * @todo 如果啥都搜不到呢？
         */                    
        if (!search_ind.empty()) {
            LocalizationPointsInfo<_FeatureT> loc_points;  
            /**
             * @todo 目前是直接提取一定范围的关键帧组合为定位地图，之后要改为，每个区域维护一个submap，
             *  直接提取这个区域的submap进行定位，这个submap也具备更新的能力 
             */
            // 遍历定位所需的所有点云标识名   将定位所需要的点云local map 提取出来 
            for (auto const& name : localize_registration_->GetUsedPointsName()) {   
                // 从数据库中查找 名字为 name 的点云 
                typename pcl::PointCloud<_FeatureT>::Ptr local_map(new pcl::PointCloud<_FeatureT>());

                for (int i = 0; i < search_ind.size(); i++) {
                    typename pcl::PointCloud<_FeatureT>::Ptr origin_points(new pcl::PointCloud<_FeatureT>());
                    //   获取该关键帧名为name的点云
                    if (!PoseGraphDataBase::GetInstance().GetKeyFramePointCloud<_FeatureT>(name, 
                            search_ind[i], origin_points)) {
                        std::cout << SlamLib::color::RED << "GetKeyFramePointCloud() error, name: " << name 
                            << std::endl;
                        throw std::bad_exception();  
                    }
                    // 读取节点的位姿
                    pcl::PointCloud<_FeatureT> trans_points;   // 转换到世界坐标系下的点云 
                    Eigen::Isometry3d pose;
                    PoseGraphDataBase::GetInstance().SearchVertexPose(search_ind[i], pose);
                    pcl::transformPointCloud(*origin_points, trans_points, pose.matrix()); // 转到世界坐标  
                    *local_map += trans_points; 
                }

                localize_registration_->SetInputSource(std::make_pair(name, local_map)); 
                loc_points.map_[name] = local_map; 
                IPC::Server::Instance().Publish("localize_map", local_map);   // 发布地图用于可视化    
            }

            loc_points.time_stamps_ = keyframe.time_stamp_; 
            localize_registration_->SetInputTarget(points);

            if (!localize_registration_->Solve(pose_in_map)) {
                LOG(WARNING)<<SlamLib::color::RED<<"错误: 定位匹配无法收敛！转换到重定位模式..."
                    <<SlamLib::color::RESET;
                work_mode_ = WorkMode::RELOCALIZATION;
                continue;  
            }
            //std::cout<<SlamLib::color::GREEN<<"after loc pose_in_map: "<<pose_in_map.matrix()<<std::endl;
            tt.toc("localization ");
            tt.tic(); 
            // 匹配评估
            typename pcl::PointCloud<_FeatureT>::ConstPtr eva_local_map(
                new pcl::PointCloud<_FeatureT>());
            std::string required_name = "filtered";    // 获取检验模块需要的点云标识名
            // 检查定位匹配阶段 是否已经构建了定位评估阶段所需要使用的地图 
            if (loc_points.map_.find(required_name) != loc_points.map_.end()) {
                eva_local_map = loc_points.map_[required_name];  
            } else {  
                // 没有所需要的地图数据则进行构建
                typename pcl::PointCloud<_FeatureT>::Ptr local_map(new pcl::PointCloud<_FeatureT>());
                
                for (int i = 0; i < search_ind.size(); i++) {
                    typename pcl::PointCloud<_FeatureT>::Ptr origin_points(new pcl::PointCloud<_FeatureT>());
                    
                    if (!PoseGraphDataBase::GetInstance().GetKeyFramePointCloud<_FeatureT>(required_name, 
                            search_ind[i], origin_points)) {
                        std::cout << SlamLib::color::RED << "错误：定位模式找不到evaluate map"
                            << SlamLib::color::RESET << std::endl;
                        throw std::bad_exception();  
                    }
                    // 读取节点的位姿
                    pcl::PointCloud<_FeatureT> trans_points;   // 转换到世界坐标系下的点云 
                    Eigen::Isometry3d pose;
                    PoseGraphDataBase::GetInstance().SearchVertexPose(search_ind[i], pose);
                    pcl::transformPointCloud(*origin_points, trans_points, pose.matrix()); // 转到世界坐标  
                    *local_map += trans_points; 
                }
                
                eva_local_map = local_map;
            }

            align_evaluator_.SetTargetPoints(eva_local_map);      // 0-15ms 
            std::pair<double, double> res = align_evaluator_.AlignmentScore(points.at(required_name), 
                                                                                pose_in_map.matrix().cast<float>(), 0.2, 0.3); // 0-10ms 
            tt.toc("evaluate ");                                                                                                          
            std::cout<<"score: "<<res.first<<std::endl;
            std::cout<<"overlap_ratio: "<<res.second<<std::endl;
            static double last_loc_record_time = -1;
            static float last_loc_record_overlap = 0;  
            // 得分大于1，重叠率太小了 < 0.3, 认为定位失败 
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
                //     *res_points = *eva_local_map;
                //         *res_points += input_transformed; 
                //     pcl::io::savePCDFileBinary(
                //         "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_all.pcd"
                //         , *res_points);
                //     pcl::io::savePCDFileBinary(
                //         "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_map.pcd"
                //         , *eva_local_map);
                //     pcl::io::savePCDFileBinary(
                //         "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_scan.pcd"
                //         , *points.at(required_name));
                //     pcl::io::savePCDFileBinary(
                //         "/home/lwh/code/lwh_ws-master/src/liv_slam-master/slam_data/point_cloud/loc_err_scan_trans.pcd"
                //         , input_transformed);
                // #endif
                continue;  
            }
            // 如果得分很低 <= 0.04  认为定位很好
            // 同时 若重叠率 < 0.8 且 距离历史关键帧的距离不太远，则认为环境有变化，此时进行地图更新
            if (res.first <= 0.15 && res.second < 0.7) {
                double min_historical_keyframe_dis = std::sqrt(search_dis.front()); 
                // 相比于历史轨迹距离足够远就直接进行建图
                if (min_historical_keyframe_dis > 10) {
                    // 转为纯建图模式
                    Eigen::Isometry3d front_pose; 
                    PoseGraphDataBase::GetInstance().SearchVertexPose(search_ind[0], front_pose); 
                    Eigen::Isometry3d relpose = front_pose.inverse() * pose_in_map; 
                    // 重新设置该关键帧的连接关系  与 约束 
                    keyframe.adjacent_id_ = search_ind[0];  
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
                     * 
                     */
                }
                // if () {
                //     // 地图更新 
                //     if (enable_map_update_) {
                //         std::cout<<SlamLib::color::GREEN<<"-----------------MAP UDAPATE!-----------------"<<std::endl;
                //         Eigen::Isometry3d front_pose; 
                //         PoseGraphDataBase::GetInstance().SearchVertexPose(search_ind[0], front_pose); 
                //         Eigen::Isometry3d relpose = front_pose.inverse() * pose_in_map; 
                //         // 重新设置该关键帧的连接关系  与 约束 
                //         keyframe.adjacent_id_ = search_ind[0];  
                //         keyframe.between_constraint_ = relpose;      // 获取该关键帧与上一关键帧之间的相对约束  last<-curr       
                //         work_mode_ = WorkMode::MAPPING;
                //         //return;  // 退出线程
                //         this->trans_odom2map_ = pose_in_map * keyframe.odom_.inverse();  
                //         //std::cout<<SlamLib::color::RED<<"update this->trans_odom2map_: "<<this->trans_odom2map_.matrix()<<std::endl;
                //         IPC::Server::Instance().Publish("odom_to_map", this->trans_odom2map_);      // 发布坐标变换
                //         continue; 
                //     }
                // }
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

            // 更新校正矩阵  
            this->trans_odom2map_ = pose_in_map * keyframe.odom_.inverse();  
            IPC::Server::Instance().Publish("odom_to_map", this->trans_odom2map_);      // 发布坐标变换
            this->keyframe_queue_sm_.lock();
            this->new_keyframe_queue_.pop_front(); 
            this->new_keyframe_points_queue_.pop_front();
            this->keyframe_queue_sm_.unlock(); 
        } 

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
        //if (work_mode_ != MAPPING) return;  
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
            SlamLib::time::TicToc tt;
            // 优化完成后 更新数据库  
            for(int i=0; i < optimizer_->GetNodeNum(); i++) {
                database.UpdateVertexPose(i, optimizer_->ReadOptimizedPose(i)); 
            }
            
            tt.toc("update dataset ");
            this->keyframe_queue_sm_.lock();  
            // 计算坐标转换矩阵
            this->trans_odom2map_ = database.GetLastVertex().pose_ 
                                                                    * database.GetLastKeyFrameData().odom_.inverse();  
            IPC::Server::Instance().Publish("odom_to_map", this->trans_odom2map_);      // 发布坐标变换
            this->keyframe_queue_sm_.unlock();  

            if (has_loop_) {
                // 进入定位模式前需要更新一下可视化 
                KeyFrameInfo<_FeatureT> keyframe_info; 
                keyframe_info.vertex_database_ = PoseGraphDataBase::GetInstance().GetAllVertex(); 
                keyframe_info.edge_database_ = PoseGraphDataBase::GetInstance().GetAllEdge(); 
                keyframe_info.new_keyframes_ = this->new_keyframe_queue_;  
                IPC::Server::Instance().Publish("keyframes_info", keyframe_info);   // 发布图关键帧  

                work_mode_ = WorkMode::LOCALIZATION; // 进入定位模式 
                std::cout << "建图线程：进入定位模式！" << std::endl;
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
    // 处理的数量  
    int num_processed = std::min<int>(this->new_keyframe_queue_.size(), 
                                                                                max_keyframes_per_update_);
    // 遍历全部关键帧队列       
    for (int i = 0; i < num_processed; i++) {
        // 从keyframe_queue中取出关键帧
        auto& keyframe = this->new_keyframe_queue_[i];
        auto& points = this->new_keyframe_points_queue_[i];  
        keyframe.id_ = PoseGraphDataBase::GetInstance().ReadVertexNum();
        Eigen::Isometry3d corrected_pose = this->trans_odom2map_ * keyframe.odom_; 
        // 把关键帧点云存储到硬盘里     不消耗内存
        for (auto iter = points.begin(); iter != points.end(); ++iter) {  
            // 存到数据库中  
            PoseGraphDataBase::GetInstance().AddKeyFramePointCloud(iter->first, 
                keyframe.id_, *(iter->second));
        }
        // 点云数据加入到回环模块进行处理
        loop_detect_->AddData(points);    // 点云
        // 添加节点  
        if (keyframe.id_ == 0) {
            // 第一个节点默认 fix
            optimizer_->AddSe3Node(corrected_pose, keyframe.id_, true);
            //  添加到数据库中   图优化中的node 和 数据库中的关键帧 序号是一一对应的
            PoseGraphDataBase::GetInstance().AddKeyFrameData(keyframe);   
            PoseGraphDataBase::GetInstance().AddVertex(keyframe.id_, corrected_pose);  
            PoseGraphDataBase::GetInstance().AddPosePoint(corrected_pose);  
            continue;
        }

        optimizer_->AddSe3Node(corrected_pose, keyframe.id_); 
        // 观测噪声
        Eigen::Matrix<double, 1, 6> noise;
        noise << 0.0025, 0.0025, 0.0025, 0.0001, 0.0001, 0.0001;
        // 如果邻接节点id 没有被设置  说明就是与上一个节点连接 
        if (keyframe.adjacent_id_ == -1) {
            keyframe.adjacent_id_ = keyframe.id_ - 1;  
        }

        optimizer_->AddSe3Edge(keyframe.adjacent_id_, keyframe.id_, keyframe.between_constraint_, noise);  
        PoseGraphDataBase::GetInstance().AddEdge(keyframe.adjacent_id_, keyframe.id_, 
                                                                                                        keyframe.between_constraint_, noise);  

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
        //  添加到数据库中  
        PoseGraphDataBase::GetInstance().AddKeyFrameData(keyframe);   
        PoseGraphDataBase::GetInstance().AddVertex(keyframe.id_, corrected_pose);  
        PoseGraphDataBase::GetInstance().AddPosePoint(corrected_pose);  
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
    // 提取所有新添加的回环数据
    std::deque<Edge>  new_loops = loop_detect_->GetNewLoops();

    if (new_loops.size() > 0) {
        has_loop_ = true;  
        
        for (uint16_t i = 0; i < new_loops.size(); i++) {
            // 添加回环边
            optimizer_->AddSe3Edge(new_loops[i].link_id_.first, new_loops[i].link_id_.second, 
                                                                    new_loops[i].constraint_, new_loops[i].noise_);  
            // 回环数据记录到数据库中
            PoseGraphDataBase::GetInstance().AddEdge(new_loops[i]);  
        }
    }
    // 如果累计的外部约束超过10个  也触发一次优化 
    // 如GNSS 约束  + 地面约束 
    if (new_external_constranit_num_ > 10) {
        do_optimize = true; 
        new_external_constranit_num_ = 0; 
    }
    // optimize the pose graph
    // 执行优化
    static int optimize_dormant = 0;
    
    if (optimize_dormant <= 0) {
        SlamLib::time::TicToc tt;
        // 如果新增足够的外部约束以及没有回环  那么不用执行全局优化 
        if (!do_optimize && !has_loop_) {
            return false;
        }  

        optimizer_->Optimize(has_loop_);  
        tt.toc("optimize ");
        do_optimize = false;  
    } else {
        optimize_dormant--;  
        return false;  
    }

    return true;  
}
}
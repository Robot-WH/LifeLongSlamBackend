/**
 * @file pose_graph_database.hpp
 * @brief   保存位姿图的数据库管理模块 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2022
 */
#pragma once 
#define NDEBUG
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <glog/logging.h>
// #include "Common/type.h"
#include "SlamLib/Common/color.hpp"
#include "SlamLib/Common/pointcloud.h"
#include "Common/keyframe.hpp"
#include "graph.hpp"
namespace lifelong_backend {
/**
 * @brief: 维护 pose graph 的数据 
 */    
class PoseGraphDataBase {
private:
    struct Info {
        uint16_t session_cnt;  //  阶段，只要重定位失败就会新建一个轨迹，session ++ 
        uint16_t trajectory_num;   // 当前数据库存在的轨迹个数  
        uint64_t keyframe_cnt;  
        uint64_t edge_cnt; 
    };
public:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 单例的创建函数  
     */            
    static PoseGraphDataBase& GetInstance() {
        static PoseGraphDataBase PoseGraph_dataBase; 
        return PoseGraph_dataBase; 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 保存数据
     * @details pose-graph的数据 保存在 session 中  
     */            
    void Save(const uint16_t& traj) {   
        // 保存keyframe信息 
        assert(database_save_path_ != ""); 
        // 保存pose-graph
        // 节点
        for (auto& item : traj_vertex_map_[traj]) {
            item.Save(database_save_path_); 
        }
        // for (uint64_t i = 0; i < vertex_container_.size(); i++) {
        //     vertex_container_[i].Save(database_save_path_);  
        // }
        // 边
        for (uint64_t i = 0; i < traj_edge_map_[traj].size(); i++) {
            traj_edge_map_[traj][i].Save(database_save_path_);  
        }
        // 更新info文件
        std::ofstream ofs(database_save_path_ + "/info");
        ofs << "session_cnt " << info_.session_cnt << "\n";
        ofs << "keyframe_cnt " << info_.keyframe_cnt << "\n";
        ofs << "edge_cnt " << info_.edge_cnt << "\n";
    }
    
    /**
     * @brief: 从指定路径中加载数据库  
     * @param database_save_path 地图空间的地址
     * @param traj 选择加载的轨迹  
     */            
    bool Load(std::string database_save_path, uint16_t traj = 0) {
        database_save_path_ = database_save_path;  
        assert(database_save_path_ != ""); 
        // 读取当前area的Info
        std::ifstream ifs(database_save_path_ + "/info");
            
        if(!ifs) {
            std::cout << "开启第一个轨迹..." << std::endl;
            info_.session_cnt = 0;  
            info_.trajectory_num = 0;  
            info_.keyframe_cnt = 0;  
            info_.edge_cnt = 0;  
            // 创建文件夹
            boost::filesystem::create_directory(database_save_path_ + "/KeyFrameDescriptor");
            boost::filesystem::create_directory(database_save_path_ + "/KeyFramePoints");
            boost::filesystem::create_directory(database_save_path_ + "/Vertex");
            boost::filesystem::create_directory(database_save_path_ + "/Edge");
            return false;
        } 

        while(!ifs.eof()) {
            std::string token;
            ifs >> token;
            
            if (token == "session_cnt") {
                ifs >> info_.session_cnt; 
                std::cout << SlamLib::color::GREEN << "session_cnt: " << info_.session_cnt <<std::endl;
            } else if (token == "trajectory_num") {
                ifs >> info_.trajectory_num; 
                std::cout << SlamLib::color::GREEN << "trajectory_cnt: " << info_.trajectory_num <<std::endl;
            } else if (token == "keyframe_cnt") {
                ifs >> info_.keyframe_cnt; 
                std::cout << "keyframe_cnt: " << info_.keyframe_cnt <<std::endl;
            } else if (token == "edge_cnt") {
                ifs >> info_.edge_cnt; 
                std::cout << "edge_cnt: " << info_.edge_cnt << SlamLib::color::RESET <<std::endl;
            }
        }

        database_vertex_info_.reserve(3000);

        if (!LoadPoseGraph(traj)) {
            DataReset(); 
            LOG(INFO) << SlamLib::color::RED<<"Load Pose-Graph failure" 
                << SlamLib::color::RESET << std::endl;
            return false; 
        }

        LOG(INFO) << SlamLib::color::GREEN<<"Load Pose-Graph done" 
            << SlamLib::color::RESET << std::endl;

        return true; 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 
     * @return true 
     * @return false 
     */
    bool LoadPoseGraph(uint16_t traj) {
        uint64_t index = 0; 
        // 遍历磁盘全部vertex数据，将属于traj轨迹的加载进来  
        while(index < info_.keyframe_cnt) {
            Vertex vertex;  
            std::ifstream ifs(database_save_path_ + "/Vertex/id_" + std::to_string(index));
            if(!ifs) {
                index++;  
                continue;
            }
            index++;  
            // 读取该结点的信息
            while(!ifs.eof()) {
                std::string token;
                ifs >> token;

                if (token == "id") {
                    ifs >> vertex.id_; 
                    //std::cout<<"vertex.id: "<<vertex.id_<<std::endl;
                } else if (token == "pose") {
                    Eigen::Matrix4d matrix; 

                    for(int i = 0; i < 4; i++) {
                        for(int j = 0; j < 4; j++) {
                            ifs >> matrix(i, j);
                        }
                    }
                    
                    vertex.pose_.translation() = matrix.block<3, 1>(0, 3);
                    vertex.pose_.linear() = matrix.block<3, 3>(0, 0);
                    //std::cout<<"vertex.pose_: "<<vertex.pose_.matrix()<<std::endl;
                } else if (token == "traj") {
                    ifs >> vertex.traj_; 
                }
            }

            database_vertex_info_.emplace_back(vertex.traj_, traj_vertex_map_[vertex.traj_].size()); 
            traj_vertex_map_[vertex.traj_].push_back(vertex); 
            
            traj_vertexDatabaseIndex_map_[vertex.traj_].push_back(vertex_database_.size());  
            vertex_database_.push_back(vertex); 
            AddPosePoint(vertex.traj_, vertex.pose_);
            
            if (vertex.traj_ != traj) {
                continue;  
            }

            // vertex_container_.emplace_back(std::move(vertex));
            AddVertex(vertex);
        }
        
        if (vertex_container_.size() == 0) {
            // std::cout<<common::RED<<
            // "Load KetFrame ERROR: vertex num keyframe num not match,  vertex num: "
            // <<vertex_container_.size()<<", keyframe num: "<<
            // keyframe_database_.size()<<std::endl;
            return false;  
        }

        index = 0;  
        // 遍历磁盘全部edge数据，将属于traj轨迹的加载进来  
        while(index < info_.edge_cnt) {
            Edge edge;  
            std::ifstream ifs(database_save_path_ + "/Edge/id_" + std::to_string(index));

            if(!ifs) {
                index++;  
                continue;;
            }

            index++;  

            while(!ifs.eof()) {
                std::string token;
                ifs >> token;

                if (token == "traj") {
                    ifs >> edge.traj_; 
                } else if (token == "id") {
                    ifs >> edge.id_; 
                    //std::cout<<"edge.id: "<<edge.id_<<std::endl;
                } else if (token == "link_head") {
                    ifs >> edge.link_id_.first; 
                    //std::cout<<"link_head: "<<edge.link_id_.first<<std::endl;
                } else if (token == "link_tail") {
                    ifs >> edge.link_id_.second; 
                    //std::cout<<"link_tail: "<<edge.link_id_.second<<std::endl;
                } else if (token == "link_head_local_index") {
                    ifs >> edge.link_head_local_index_; 
                } else if (token == "constraint") {
                    Eigen::Matrix4d matrix; 

                    for(int i = 0; i < 4; i++) {
                        for(int j = 0; j < 4; j++) {
                            ifs >> matrix(i, j);
                        }
                    }

                    edge.constraint_.translation() = matrix.block<3, 1>(0, 3);
                    edge.constraint_.linear() = matrix.block<3, 3>(0, 0);
                    //std::cout<<"edge.constraint_: "<<edge.constraint_.matrix()<<std::endl;
                } else if (token == "noise") {
                    for(int i = 0; i < 6; i++) {
                            ifs >> edge.noise_(0, i);
                    }
                    //std::cout<<"edge.noise_: "<<edge.noise_.matrix()<<std::endl;
                }
            }

            traj_edge_map_[edge.traj_].push_back(edge); 

            if (edge.traj_ != traj) {
                continue;  
            }

            edge_container_.push_back(std::move(edge));
        }

        return true; 
    }

    uint16_t CreateNewSession() {
        ++info_.session_cnt;
        traj_vertex_map_[info_.session_cnt].reserve(1000);
        traj_edge_map_[info_.session_cnt].reserve(1000);
        return info_.session_cnt;
    }

    /**
     * @brief Set the Save Path object
     * @param  database_save_pathMy Param doc
     */
    void SetSavePath(std::string const& database_save_path) {
        database_save_path_ = database_save_path; 
    }

    
    /**
     * @brief 添加一个关键帧的数据
     * @param  keyframe  关键帧
     */

    /**
     * @brief 
     * 
     * @tparam _PointT 
     * @param traj 
     * @param corrected_pose 
     * @param keyframe_points 
     * @return std::pair<uint32_t, uint32_t>  vertex的 <全局id，轨迹的局部Index>
     */
    template<typename _PointT>
    std::pair<uint32_t, uint32_t> AddKeyFrame(uint16_t const& traj, Eigen::Isometry3d const& corrected_pose,
                                            SlamLib::FeaturePointCloudContainer<_PointT> const& keyframe_points) {
        std::pair<uint32_t, uint32_t> res;       // <global id, local index>
        has_new_keyframe_ = true; 
        // 添加节点
        res.second = AddVertex(info_.keyframe_cnt, traj, corrected_pose);     // 返回结点位于轨迹中的局部index 
        // 把关键帧点云存储到硬盘里     不消耗内存
        for (auto iter = keyframe_points.begin(); iter != keyframe_points.end(); ++iter) {  
            std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                + iter->first + std::to_string(info_.keyframe_cnt) + ".pcd";
            pcl::io::savePCDFileBinary(file_path, *(iter->second));
        }

        res.first = info_.keyframe_cnt;
        ++info_.keyframe_cnt;
        return res;  
    }
    
    /**
     * @brief 添加一个位姿图节点
     * 
     * @param id 
     * @param traj 
     * @param pose 
     * @return uint32_t 位于当前轨迹的index 
     */
    uint32_t AddVertex(uint64_t const& id, uint16_t const& traj, Eigen::Isometry3d const& pose) {
        uint32_t local_index = traj_vertex_map_[traj].size();
        database_vertex_info_.emplace_back(traj, local_index); 
        traj_vertex_map_[traj].emplace_back(id, traj, pose);
        AddPosePoint(traj, pose);  
        return local_index;  
    }

    /**
     * @brief 重载 
     * 
     * @param vertex 
     */
    void AddVertex(Vertex const& vertex) {
        global_to_local_ID_[vertex.id_] = vertex_container_.size();  
        vertex_container_.push_back(vertex);
        AddPosePoint(vertex.pose_);  
    }

    
    /**
     * @brief 添加一个位姿到位姿点云 
     * @param  pose             My Param doc
     */
    void AddPosePoint(Eigen::Isometry3d const& pose) {
        pose_cloud_mt_.lock();  
        //  将位置数据保存到位置点云
        pcl::PointXYZ Position3D;
        Position3D.x = pose.translation().x();
        Position3D.y = pose.translation().y();
        Position3D.z = pose.translation().z();
        cloudKeyFramePosition3D_->push_back(Position3D);
        // 保存姿态数据到位姿点云
        pcl::PointXYZI Rot3D; 
        Eigen::Quaterniond q(pose.rotation());
        Rot3D.x = q.x();
        Rot3D.y = q.y();
        Rot3D.z = q.z();
        Rot3D.intensity = q.w();  
        cloudKeyFrameRot3D_->push_back(Rot3D);
        pose_cloud_mt_.unlock();  
    }

    /**
     * @brief 
     * 
     * @param traj 
     * @param pose 
     */
    void AddPosePoint(uint16_t traj, Eigen::Isometry3d const& pose) {
        if (traj_keyframePositionCloud_.find(traj) == traj_keyframePositionCloud_.end()) {
            traj_keyframePositionCloud_[traj] = 
                pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            traj_keyframeRotCloud_[traj] = 
                pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        }
        pose_cloud_mt_.lock();  
        //  将位置数据保存到位置点云
        pcl::PointXYZ Position3D;
        Position3D.x = pose.translation().x();
        Position3D.y = pose.translation().y();
        Position3D.z = pose.translation().z();
        traj_keyframePositionCloud_[traj]->push_back(Position3D);
        // 保存姿态数据到位姿点云
        pcl::PointXYZI Rot3D; 
        Eigen::Quaterniond q(pose.rotation());
        Rot3D.x = q.x();
        Rot3D.y = q.y();
        Rot3D.z = q.z();
        Rot3D.intensity = q.w();  
        traj_keyframeRotCloud_[traj]->push_back(Rot3D);
        pose_cloud_mt_.unlock();  
    }

    /**
     * @brief 添加一个位姿图边 
     * @param  head_id  首节点id
     * @param  tail_id    尾节点id
     * @param  constraint   边位姿约束
     * @param  noise    约束协方差
     */
    inline void AddEdge(int16_t const& traj, uint64_t const& head_id, uint64_t const& tail_id, 
            const uint32_t& link_head_local_index, Eigen::Isometry3d const& constraint, 
            Eigen::Matrix<double, 1, 6> const& noise) {
        traj_edge_map_[traj].emplace_back(traj, info_.edge_cnt, head_id, tail_id, link_head_local_index, constraint, noise);
        ++info_.edge_cnt;
    }

    /**
     * @brief 重载
     * 
     * @param edge 
     */
    inline void AddEdge(uint16_t const& traj, Edge& edge, bool new_edge = true) {
        if (new_edge) {
            edge.id_ = info_.edge_cnt; 
            ++info_.edge_cnt;
        }
        edge.traj_ = traj;  
        traj_edge_map_[traj].push_back(edge);
    }

    /**
     * @brief 读取当前posegraph中节点的数量 
     * 
     * @return uint64_t 
     */
    uint64_t GetGraphVertexNum() {
        database_mt_.lock_shared();
        uint64_t num = vertex_container_.size();
        database_mt_.unlock_shared();
        return num;  
    }

    /**
     * @brief Get the Trajector Vertex Number object  
     * 
     * @param traj  轨迹id 
     * @return uint64_t 
     */
    uint64_t GetTrajectorVertexNum(uint16_t traj) {
        database_mt_.lock_shared();
        uint64_t num = traj_vertex_map_[traj].size();
        database_mt_.unlock_shared();
        return num;  
    }

    /**
     * @brief Get the Last Vertex object  
     * @return Vertex 
     */
    Vertex GetLastVertex() {
        Vertex vertex;  
        boost::shared_lock<boost::shared_mutex> lock(database_mt_); 

        if (vertex_container_.empty()) {
            return vertex;
        }

        vertex = vertex_container_.back();  
        return vertex;  
    }

    /**
     * @brief Get the Vertex By ID object
     * 
     * @return Vertex 
     */
    Vertex GetVertexByID(uint64_t id) const {
        // 如果在内存中直接读取
        // if (global_to_local_ID_.find(id) != global_to_local_ID_.end()) {
        //     return vertex_container_[global_to_local_ID_.at(id)];
        // }
        // 读取磁盘  构造vertex 
        Vertex vertex; 
        std::ifstream ifs(database_save_path_ + "/Vertex/id_" + std::to_string(id));

        if(!ifs) {
            return vertex;
        }

        while(!ifs.eof()) {
            std::string token;
            ifs >> token;

            if (token == "id") {
                ifs >> vertex.id_; 
                //std::cout<<"vertex.id: "<<vertex.id_<<std::endl;
            } else if (token == "pose") {
                Eigen::Matrix4d matrix; 

                for(int i = 0; i < 4; i++) {
                    for(int j = 0; j < 4; j++) {
                        ifs >> matrix(i, j);
                    }
                }

                vertex.pose_.translation() = matrix.block<3, 1>(0, 3);
                vertex.pose_.linear() = matrix.block<3, 3>(0, 0);
                //std::cout<<"vertex.pose_: "<<vertex.pose_.matrix()<<std::endl;
            } else if (token == "traj") {
                ifs >> vertex.traj_; 
            }
        }

        return vertex;  
    }

    /**
     * @brief Get the Vertex By Index object
     *                  从数据库中按索引获取vertex 
     * @param index 
     * @return Vertex 
     */
    Vertex GetVertexByDatabaseIndex(const uint64_t& index) const {
        // return vertex_database_[index];
        auto& vertex_info = database_vertex_info_[index];
        return traj_vertex_map_.at(vertex_info.first)[vertex_info.second];
    }

    /**
     * @brief Get the Vertex By Local Index object
     *                  
     * @param index 
     * @return Vertex 
     */
    Vertex GetVertexByTrajectoryLocalIndex(const uint16_t& traj, const uint64_t& index) const {
        // uint64_t database_idx = traj_vertexDatabaseIndex_map_.at(traj)[index];  
        // return vertex_database_[database_idx];
        return traj_vertex_map_.at(traj)[index];  
    }

    /**
     * @brief Get the Last Key Frame Data object
     * 
     * @return KeyFrame 
     */
    KeyFrame GetLastKeyFrameData() {
        KeyFrame keyframe;  
        boost::shared_lock<boost::shared_mutex> lock(database_mt_); 

        if (keyframe_database_.empty()) {
            return keyframe;
        }

        keyframe = keyframe_database_.back();  
        return keyframe;  
    }

    /**
     * @brief Get the New Key Frame object
     * 
     * @param keyframe 
     * @return true 
     * @return false 
     */
    bool GetNewKeyFrame(KeyFrame &keyframe) {
        if (!has_new_keyframe_) {
            return false;  
        }

        boost::shared_lock<boost::shared_mutex> lock(database_mt_); 
        keyframe = keyframe_database_.back();  
        has_new_keyframe_ = false; 
        return true;  
    }


    /**
     * @brief Get the All Vertex object
     * 
     * @return std::deque<Vertex> const& 
     */
    std::deque<Vertex>& GetAllVertex() {
        return vertex_container_;
    }

    /**
     * @brief Get the Trajectory Vertex object
     * 
     * @param traj 
     * @return std::deque<Vertex>& 
     */
    std::vector<Vertex>& GetTrajectoryVertex(const uint16_t& traj) {
        return traj_vertex_map_[traj];
    }

    /**
     * @brief 
     * 
     * @param session 
     * @return std::deque<Vertex> const& 
     */
    void GetTrajectoryVertex(uint16_t const& traj, std::deque<Vertex>& traj_vertexs) {
        Vertex vertex;  
        uint64_t index = 0; 

        while(1) {
            std::ifstream ifs(database_save_path_ + "/Vertex/id_" + std::to_string(index));
            
            if(!ifs) {
                break;
            }

            index++;  

            while(!ifs.eof()) {
                std::string token;
                ifs >> token;

                if (token == "id") {
                    ifs >> vertex.id_; 
                    //std::cout<<"vertex.id: "<<vertex.id_<<std::endl;
                } else if (token == "pose") {
                    Eigen::Matrix4d matrix; 

                    for(int i = 0; i < 4; i++) {
                        for(int j = 0; j < 4; j++) {
                            ifs >> matrix(i, j);
                        }
                    }
                    
                    vertex.pose_.translation() = matrix.block<3, 1>(0, 3);
                    vertex.pose_.linear() = matrix.block<3, 3>(0, 0);
                    //std::cout<<"vertex.pose_: "<<vertex.pose_.matrix()<<std::endl;
                } else if (token == "traj") {
                    ifs >> vertex.traj_; 
                }
            }

            if (vertex.traj_ != traj) {
                continue;  
            }

            traj_vertexs.emplace_back(std::move(vertex));
        }
    }

    /**
     * @brief Get the Session Edge object  获取某一个轨迹的所有边   
     * 
     * @param session 
     * @param session_vertexs 
     */
    void GetTrajectoryEdge(uint16_t const& traj, std::deque<Edge>& traj_edges) {
        Edge edge;  
        uint64_t index = 0; 

        while(1) {
            std::ifstream ifs(database_save_path_ + "/Edge/id_" + std::to_string(index));
            
            if(!ifs) {
                break;
            }

            index++;  

            while(!ifs.eof()) {
                std::string token;
                ifs >> token;

                if (token == "traj") {
                    ifs >> edge.traj_; 

                    if (edge.traj_ != traj) {
                        break;
                    }
                } else if (token == "id") {
                    ifs >> edge.id_; 
                    //std::cout<<"edge.id: "<<edge.id_<<std::endl;
                } else if (token == "link_head") {
                    ifs >> edge.link_id_.first; 
                    //std::cout<<"link_head: "<<edge.link_id_.first<<std::endl;
                } else if (token == "link_tail") {
                    ifs >> edge.link_id_.second; 
                    //std::cout<<"link_tail: "<<edge.link_id_.second<<std::endl;
                } else if (token == "constraint") {
                    Eigen::Matrix4d matrix; 

                    for(int i = 0; i < 4; i++) {
                        for(int j = 0; j < 4; j++) {
                            ifs >> matrix(i, j);
                        }
                    }

                    edge.constraint_.translation() = matrix.block<3, 1>(0, 3);
                    edge.constraint_.linear() = matrix.block<3, 3>(0, 0);
                    //std::cout<<"edge.constraint_: "<<edge.constraint_.matrix()<<std::endl;
                } else if (token == "noise") {
                    for(int i = 0; i < 6; i++) {
                            ifs >> edge.noise_(0, i);
                    }
                    //std::cout<<"edge.noise_: "<<edge.noise_.matrix()<<std::endl;
                }
            }

            if (edge.traj_ == traj) {
                traj_edges.push_back(edge);
            }
        }
    }

    /**
     * @brief Get the Trajectory Edge object
     * 
     * @param traj 
     * @return const std::deque<Edge>& 
     */
    const auto& GetTrajectoryEdge(const uint16_t& traj) {
        return traj_edge_map_[traj];
    }

    /**
     * @brief Get the All Edge object
     * @return std::deque<Edge> const& 
     */
    std::deque<Edge> const& GetAllEdge() {
        return edge_container_;
    }

    /**
     * @brief Get the Key Frame Position Cloud object 获取位置点云
     * 
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetKeyFramePositionCloud() {
        boost::shared_lock<boost::shared_mutex> lock(pose_cloud_mt_); 
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>(*cloudKeyFramePosition3D_));
    }

    /**
     * @brief Get the Key Frame Position Cloud object
     * 
     * @param traj 轨迹的id
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr 
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetKeyFramePositionCloud(uint16_t traj) {
        boost::shared_lock<boost::shared_mutex> lock(pose_cloud_mt_); 
        return traj_keyframePositionCloud_[traj];
    }

    /**
     * @brief Get the Key Frame Rot Cloud object获取姿态点云
     * 
     * @return pcl::PointCloud<pcl::PointXYZI>::Ptr 
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr GetKeyFrameRotCloud() {
        boost::shared_lock<boost::shared_mutex> lock(pose_cloud_mt_); 
        return pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>(*cloudKeyFrameRot3D_));
    }

    /**
     * @brief: 获取以一个节点为中心，前后若干个连续相邻节点共同组成的local map 
     *  @details 注意，由于Multi-session,数据库中连续的若干帧不一定真正在时序上连续 
     * @param center_id localmap的中心节点 id 
     * @param neighbors_num 中心节点前/后 邻居个数 
     * @param points_name 点云的标识名
     * @param[out] local_map 构造的local map 
     * @return 是否成功
     */            
    template<typename _PointT>
    bool GetAdjacentLinkNodeLocalMap(uint64_t const& center_id, uint16_t const& neighbors_num, 
                                                                                std::string const& points_name, 
                                                                                typename pcl::PointCloud<_PointT>::ConstPtr& local_map) {
        pcl::PointCloud<_PointT> origin_points;   // 激光坐标系下的点云
        pcl::PointCloud<_PointT> trans_points;   // 转换到世界坐标系下的点云 
        typename pcl::PointCloud<_PointT>::Ptr map(new pcl::PointCloud<_PointT>()); 
        Vertex center_vertex = GetVertexByID(center_id);

        for (int16_t i = -neighbors_num; i <= neighbors_num; i++ ) {
            int64_t curr_id = center_id + i;
            std::cout << "curr_id: " << curr_id << std::endl;
            Vertex curr_vertex = GetVertexByID(curr_id);
            // 处理边界
            if (curr_id < 0) {
                curr_id += 2 * neighbors_num + 1;
                std::cout << "curr_id < 0, adjust: " << curr_id << std::endl;
            }else if (curr_id >= info_.keyframe_cnt) {
                curr_id -= (2 * neighbors_num + 1); 
                std::cout << "curr_id >= info_.keyframe_num, adjust: " << curr_id << std::endl;
            } else if (center_vertex.traj_ != curr_vertex.traj_) {
                // 进入不同的session了 
                if (i >= 0) {
                    curr_id -= (2 * neighbors_num + 1); 
                    std::cout << "center_vertex.traj_ != curr_vertex.traj_, i >= 0, adjust: " << curr_id << std::endl;
                } else {
                    curr_id += (2 * neighbors_num + 1);
                    std::cout << "center_vertex.session_ != curr_vertex.session_, i < 0, adjust: " << curr_id << std::endl;
                }
            }

            std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                + points_name + std::to_string(curr_id) + ".pcd";

            if (pcl::io::loadPCDFile(file_path, origin_points) < 0) {
                return false;
            }

            pcl::transformPointCloud (origin_points, trans_points, curr_vertex.pose_.matrix()); // 转到世界坐标  
            *map += trans_points; 
        }

        local_map = map;  
        return true;  
    }
    
    /**
     * @brief Get the Localmap From Trajectory Node Index object
     *                      将指定轨迹的指定index结点的点云拼接成local map  
     * @tparam _PointT 
     * @param traj 选择的轨迹
     * @param search_ind 用来拼接点云的结点的序号 
     * @param points_name 选择的点云表示名
     * @param local_map 
     * @return true 
     * @return false 
     */
    template<typename _PointT>
    bool GetLocalmapFromTrajectoryNodeIndex(uint16_t traj, std::vector<int> const& search_ind,
            std::string const& points_name, typename pcl::PointCloud<_PointT>::Ptr& local_map) {
        pcl::PointCloud<_PointT> origin_points;   // 激光坐标系下的点云
        pcl::PointCloud<_PointT> trans_points;   // 转换到世界坐标系下的点云 
        typename pcl::PointCloud<_PointT>::Ptr map(new pcl::PointCloud<_PointT>()); 
        // 遍历每一个index的结点
        for (const int& idx : search_ind ) {
            uint64_t database_idx = traj_vertexDatabaseIndex_map_[traj][idx];  
            Vertex vertex = vertex_database_[database_idx];

            std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                + points_name + std::to_string(vertex.id_) + ".pcd";

            if (pcl::io::loadPCDFile(file_path, origin_points) < 0) {
                return false;
            }

            pcl::transformPointCloud (origin_points, trans_points, vertex.pose_.matrix()); // 转到世界坐标  
            *map += trans_points; 
        }

        local_map = map;  
        return true;  
    }

    /**
     * @brief Get the Nearby Trajectory Node Local Map object
     * 
     * @tparam _PointT 
     * @param index 
     * @param neighbors_num 
     * @param points_name 
     * @param local_map 
     * @return true 
     * @return false 
     */
    template<typename _PointT>
    bool GetNearbyTrajectoryNodeLocalMapFromGlobalIndex(
            uint64_t const& index, uint16_t const& neighbors_num, 
            std::string const& points_name, 
            typename pcl::PointCloud<_PointT>::ConstPtr& local_map) {
        const Vertex& curr_vertex = vertex_database_[index];

    }

    /**
     * @brief: 获取keyframe 的一个 名字为 name 的点云数据 
     * @param name 点云名称
     * @param global_id 关键帧在数据库中的全局id
     * @param[out] curr_points 读取的结果 
     * @return 
     */            
    template<typename _PointT>
    bool GetKeyFramePointCloud(std::string const& name, uint32_t const& global_id, 
            typename pcl::PointCloud<_PointT>::Ptr& curr_points) {
        std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                + name + std::to_string(global_id) + ".pcd";

        if (pcl::io::loadPCDFile(file_path, *curr_points) < 0) {
            return false;
        }

        return true;  
    }

    template<typename _PointT>
    bool GetKeyFramePointCloud(std::string const& name, uint32_t const& global_id, 
            pcl::PointCloud<_PointT>& curr_points) {
        curr_points.clear();  
        std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                + name + std::to_string(global_id) + ".pcd";

        if (pcl::io::loadPCDFile(file_path, curr_points) < 0) {
            return false;
        }

        return true;  
    }

    /**
     * @brief Get the DataBase Info object
     * 
     * @return const Info& 
     */
    const Info& GetDataBaseInfo() const {
        return info_;  
    }

    /**
     * @brief Get the Local ID object   即Pose-graph中的node id  
     * 
     * @param global_id 
     * @return uint64_t 
     */
    uint64_t GetLocalID(uint64_t const& global_id) {
        return global_to_local_ID_.at(global_id);
    }

    /**
     * @brief: 根据id获取vertex的pose 
     * @param id
     */            
    inline bool SearchVertexPose(uint64_t const& id, Eigen::Isometry3d &pose) const {
        Vertex v = GetVertexByID(id); 
        pose = v.pose_;
        return true;  
    }

    /**
     * @brief 合并两个轨迹
     *      将target 合并到source 
     * @param source_traj 
     * @param target_traj 
     */
    void MergeTrajectory(uint16_t source_traj, uint16_t target_traj) {
        // std::cout << "MergeTrajectory, source_traj: " << source_traj << ", target_traj: " << target_traj
        //     << std::endl;
        // std::cout << "合并前，vertex size: " << traj_vertex_map_[source_traj].size() << std::endl;
        uint32_t source_traj_size = traj_vertex_map_[source_traj].size();
        traj_vertex_map_[source_traj].insert(traj_vertex_map_[source_traj].end(), 
                                                                                        traj_vertex_map_[target_traj].begin(),
                                                                                        traj_vertex_map_[target_traj].end());
        // std::cout << "合并后，vertex size: " << traj_vertex_map_[source_traj].size() << std::endl;
        // 更新info
        for (auto& info : database_vertex_info_) {
            if (info.first == target_traj) {
                info.first = source_traj;
                info.second += source_traj_size;
            }
        }

        for (auto& edge : traj_edge_map_[target_traj]) {
            edge.traj_ = source_traj; 
            edge.link_head_local_index_ += source_traj_size;
        }

        traj_edge_map_[source_traj].insert(traj_edge_map_[source_traj].end(), 
                                                                                traj_edge_map_[target_traj].begin(),
                                                                                traj_edge_map_[target_traj].end());
        // 合并位姿点云     
        *traj_keyframePositionCloud_[source_traj] += *traj_keyframePositionCloud_[target_traj];
        *traj_keyframeRotCloud_[source_traj] += *traj_keyframeRotCloud_[target_traj];
        // 删除target
        traj_vertex_map_.erase(target_traj); 
        traj_edge_map_.erase(target_traj); 
        traj_keyframeRotCloud_.erase(target_traj);
        traj_keyframePositionCloud_.erase(target_traj);
    }

    /**
     * @brief 
     * 
     * @param id 
     * @param correct_pose 
     */
    // inline void UpdateVertexPose(uint32_t id, Eigen::Isometry3d correct_pose) {
    //     boost::unique_lock<boost::shared_mutex> lock1(database_mt_);    // 写锁 
    //     boost::unique_lock<boost::shared_mutex> lock2(pose_cloud_mt_);    // 写锁 
    //     vertex_container_[id].SetPose(correct_pose);  // 节点 位姿更新  
    //     // 位姿点云更新   
    //     cloudKeyFramePosition3D_->points[id].x = correct_pose.translation().x();
    //     cloudKeyFramePosition3D_->points[id].y = correct_pose.translation().y();
    //     cloudKeyFramePosition3D_->points[id].y = correct_pose.translation().y();
        
    //     Eigen::Quaterniond q(correct_pose.rotation());
    //     cloudKeyFrameRot3D_->points[id].x = q.x();
    //     cloudKeyFrameRot3D_->points[id].y = q.y();
    //     cloudKeyFrameRot3D_->points[id].z = q.z();
    //     cloudKeyFrameRot3D_->points[id].intensity = q.w();   
    // }

    /**
     * @brief 
     * 
     */
    void DataReset() {
        cloudKeyFramePosition3D_->clear();
        cloudKeyFrameRot3D_->clear();
        keyframe_database_.clear();
        edge_container_.clear();
        vertex_container_.clear(); 
    }

protected:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PoseGraphDataBase() {
        cloudKeyFramePosition3D_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>()
        ); 
        cloudKeyFrameRot3D_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>()
        ); 
        database_save_path_ = ""; 
    }

    PoseGraphDataBase(PoseGraphDataBase const& object) {}
    PoseGraphDataBase(PoseGraphDataBase&& object) {}

private:
    Info info_;
    bool has_new_keyframe_ = false; 
    std::string database_save_path_;  
    std::string session_path_; 
    boost::shared_mutex pose_cloud_mt_, database_mt_, loop_mt_;  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyFramePosition3D_;  // 历史关键帧位置
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudKeyFrameRot3D_; // 历史关键帧姿态   四元数形式  
    std::unordered_map<uint16_t, pcl::PointCloud<pcl::PointXYZ>::Ptr> traj_keyframePositionCloud_;
    std::unordered_map<uint16_t, pcl::PointCloud<pcl::PointXYZI>::Ptr> traj_keyframeRotCloud_;

    /**
     * @todo  没什么用  准备删除 
     */
    std::deque<KeyFrame> keyframe_database_; // 保存全部关键帧的观测信息   
    

    std::vector<std::pair<uint16_t, uint32_t>> database_vertex_info_;     // <traj，local_index>
    std::unordered_map<uint16_t, std::vector<Vertex>> traj_vertex_map_;
    std::unordered_map<uint16_t, std::deque<uint64_t>> traj_vertexDatabaseIndex_map_;
    std::unordered_map<uint16_t, std::vector<Edge>> traj_edge_map_;

    /**
     * @todo 尝试下面的数据结构  
     * 
     */
    std::vector<Vertex> vertex_database_;    // 缓存数据库中全部结点信息 
    std::unordered_map<uint16_t, std::list<uint32_t>> traj_vertex_list_;
    std::unordered_map<uint16_t, std::list<Edge>> traj_edge_list_;


    std::deque<Edge> edge_container_; // 保存图节点
    std::deque<Vertex> vertex_container_; // 保存图的边
    std::unordered_map<uint64_t, uint64_t> global_to_local_ID_;  // 数据库id到位姿图id的映射  
}; // class 
} // namespace 
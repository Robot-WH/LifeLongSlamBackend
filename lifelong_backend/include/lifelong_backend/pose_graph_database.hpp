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
#include "SlamLib/tic_toc.hpp"
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
        uint64_t last_keyframe_id;  
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
    void Save() {   
        // 保存keyframe信息 
        assert(traj_space_path_ != ""); 
        std::cout << "traj_space_path_: " << traj_space_path_ << std::endl;
        SlamLib::time::TicToc tt;
        // 保存pose-graph 节点
        // 保存每一条轨迹的每一个node 
        for (const auto& item : traj_vertex_map_) {
            for (const auto& vertex : item.second) {
                vertex.Save(traj_space_path_); 
            }
        }
        tt.toc("save vertex ");
        tt.tic();  
        // 边
        for (const auto& item : traj_edge_map_) {
            for (const auto& edge : item.second) {
                edge.Save(traj_space_path_); 
            }
        }
        tt.toc("save edge ");
        // 更新info文件
        std::ofstream ofs(traj_space_path_ + "/info");
        ofs << "session_cnt " << info_.session_cnt << "\n";
        ofs << "trajectory_num " << traj_vertex_map_.size() << "\n";
        ofs << "last_keyframe_id " << info_.last_keyframe_id << "\n";    // last_keyframe_id 比当前保存下来的最后一个关键帧的id大1
        ofs << "edge_cnt " << info_.edge_cnt << "\n";
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 从指定路径中加载数据库  
     * @param traj_space_path 地图空间的地址
     * @param traj 选择加载的轨迹  
     */            
    bool Load(std::string traj_space_path, uint16_t traj = 0) {
        traj_space_path_ = traj_space_path;  
        assert(traj_space_path_ != ""); 
        // 读取当前轨迹空间的Info
        std::ifstream ifs(traj_space_path_ + "/info");
        DataReset(); 
        // 信息为空  说明是第一次建图
        if(!ifs) {
            std::cout << "开启第一个轨迹..." << std::endl;
            info_.session_cnt = 0;  
            info_.trajectory_num = 0;  
            info_.last_keyframe_id = 0;  
            info_.edge_cnt = 0;  
            // 创建文件夹
            boost::filesystem::create_directory(traj_space_path_ + "/KeyFrameDescriptor");
            boost::filesystem::create_directory(traj_space_path_ + "/KeyFramePoints");
            boost::filesystem::create_directory(traj_space_path_ + "/Vertex");
            boost::filesystem::create_directory(traj_space_path_ + "/Edge");
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
            } else if (token == "last_keyframe_id") {
                ifs >> info_.last_keyframe_id; 
                std::cout << "last_keyframe_id: " << info_.last_keyframe_id <<std::endl;
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
        uint64_t curr_id = 0; 
        std::vector<uint32_t> vertex_localIndex(info_.last_keyframe_id, 0);
        // 遍历磁盘全部vertex数据，将属于traj轨迹的加载进来  
        SlamLib::time::TicToc tt;
        while(curr_id < info_.last_keyframe_id) {
            Vertex vertex;  
            if (!vertex.Load(traj_space_path_ + "/Vertex/id_" + std::to_string(curr_id))) {
                curr_id++;  
                continue;
            }
            curr_id++;  
            database_vertex_info_.emplace_back(vertex.traj_, traj_vertex_map_[vertex.traj_].size()); 
            vertex_localIndex[vertex.id_] = traj_vertex_map_[vertex.traj_].size();  
            traj_vertex_map_[vertex.traj_].push_back(vertex); 
            AddPosePoint(vertex.traj_, vertex.pose_);
            if (vertex.traj_ != traj) {
                continue;  
            }
            AddVertex(vertex);
        }
        tt.toc("load vertex: ");

        // if (vertex_container_.size() == 0) {
        //     return false;
        // }

        curr_id = 0;  
        tt.tic(); 
        // 遍历磁盘全部edge数据，将属于traj轨迹的加载进来  
        while(curr_id < info_.edge_cnt) {
            Edge edge;  
            if (!edge.Load(traj_space_path_ + "/Edge/id_" + std::to_string(curr_id))) {
                curr_id++;  
                continue;
            }
            curr_id++;  
            edge.link_head_local_index_ = vertex_localIndex[edge.link_id_.first]; 
            traj_edge_map_[edge.traj_].push_back(edge); 
        }
        tt.toc("load edge ");
        return true; 
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Create a New Session object
     * 
     * @return uint16_t 
     */
    uint16_t CreateNewSession() {
        traj_vertex_map_[info_.session_cnt].reserve(1000);
        traj_edge_map_[info_.session_cnt].reserve(1000);
        ++info_.session_cnt;
        return info_.session_cnt - 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    std::pair<uint32_t, uint32_t> 
    AddKeyFrame(uint16_t const& traj, Eigen::Isometry3d const& corrected_pose,
                                    SlamLib::FeaturePointCloudContainer<_PointT> const& keyframe_points) {
        std::pair<uint32_t, uint32_t> res;       // <global id, local index>
        has_new_keyframe_ = true; 
        // 添加节点
        res.second = AddVertex(info_.last_keyframe_id, traj, corrected_pose);     // 返回结点位于轨迹中的局部index 
        // 把关键帧点云存储到硬盘里     不消耗内存
        for (auto iter = keyframe_points.begin(); iter != keyframe_points.end(); ++iter) {  
            std::string file_path = traj_space_path_ + "/KeyFramePoints/key_frame_" 
                + iter->first + std::to_string(info_.last_keyframe_id) + ".pcd";
            pcl::io::savePCDFileBinary(file_path, *(iter->second));
        }
        res.first = info_.last_keyframe_id;
        ++info_.last_keyframe_id;
        return res;  
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        // std::cout << "AddVertex, traj: " << traj << ",id: " << id << std::endl;
        AddPosePoint(traj, pose);  
        return local_index;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 重载 
     * 
     * @param vertex 
     */
    void AddVertex(const Vertex& vertex) {
        vertex_container_.push_back(vertex);
        AddPosePoint(vertex.pose_);  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        }
        pose_cloud_mt_.lock();  
        //  将位置数据保存到位置点云
        pcl::PointXYZ Position3D;
        Position3D.x = pose.translation().x();
        Position3D.y = pose.translation().y();
        Position3D.z = pose.translation().z();
        traj_keyframePositionCloud_[traj]->push_back(Position3D);
        pose_cloud_mt_.unlock();  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 添加一个位姿图边 
     * @param  head_id  首节点id
     * @param  tail_id    尾节点id
     * @param  constraint   边位姿约束
     * @param  noise    约束协方差
     */
    inline void AddEdge(const int16_t& traj, const uint64_t& head_id, const uint64_t& tail_id, 
            const uint32_t& link_head_local_index, const Eigen::Isometry3d& constraint, 
            const Eigen::Matrix<double, 1, 6>& noise) {
        traj_edge_map_[traj].emplace_back(traj, info_.edge_cnt, head_id, tail_id, 
                                                                                        link_head_local_index, constraint, noise);
        ++info_.edge_cnt;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 重载
     * 
     * @param edge 
     */
    inline void AddEdge(const uint16_t& traj, Edge& edge, bool new_edge = true) {
        if (new_edge) {
            edge.id_ = info_.edge_cnt; 
            ++info_.edge_cnt;
        }
        edge.traj_ = traj;  
        traj_edge_map_[traj].push_back(edge);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 
     * 
     * @param traj_id 
     * @return true 
     * @return false 
     */
    bool FindTrajectory(uint16_t traj_id) {
        if (traj_vertex_map_.find(traj_id) == traj_vertex_map_.end()) {
            return false;
        }
        return true;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 读取当前posegraph中节点的数量 
     * 
     * @return uint64_t 
     */
    // uint64_t GetGraphVertexNum() {
    //     database_mt_.lock_shared();
    //     uint64_t num = vertex_container_.size();
    //     database_mt_.unlock_shared();
    //     return num;
    // }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Trajector Vertex Number object  
     * 
     * @param traj  轨迹id 
     * @return uint64_t 
     */
    uint64_t GetTrajectorVertexNum(uint16_t traj) {
        database_mt_.lock_shared();
        // std::cout << "GetTrajectorVertexNum" << std::endl;
        uint64_t num = traj_vertex_map_[traj].size();
        // std::cout << "num: " << num << std::endl;
        database_mt_.unlock_shared();
        return num;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Vertex By ID object
     * 
     * @return Vertex 
     */
    Vertex GetVertexByID(uint64_t id) const {
        // 读取磁盘  构造vertex 
        Vertex vertex; 
        std::ifstream ifs(traj_space_path_ + "/Vertex/id_" + std::to_string(id));

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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Vertex By Index object
     *                  从数据库中按索引获取vertex 
     * @param global_index 
     * @return Vertex   11
     */
    Vertex GetVertexByGlobalIndex(const uint64_t& global_index) const {
        // std::cout << "GetVertexByGlobalIndex" << std::endl;
        auto& vertex_info = database_vertex_info_[global_index]; // 获得该vertex <轨迹id，该vertex在轨迹中的局部index> 
        return traj_vertex_map_.at(vertex_info.first)[vertex_info.second];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Vertex By Local Index object
     *                  
     * @param local_index 
     * @return Vertex 
     */
    Vertex GetVertexByTrajectoryLocalIndex(const uint16_t& traj, const uint64_t& local_index) const {
        // std::cout << "GetVertexByTrajectoryLocalIndex, index: " << local_index << std::endl;
        return traj_vertex_map_.at(traj)[local_index];  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Trajectory Vertex object
     * 
     * @param traj 
     * @return std::deque<Vertex>&     111
     */
    std::vector<Vertex>& GetTrajectoryVertex(const uint16_t& traj) {
        return traj_vertex_map_.at(traj);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 
     * 
     * @param session 
     * @return std::deque<Vertex> const& 
     */
    void GetTrajectoryVertex(const uint16_t& traj, std::deque<Vertex>& traj_vertexs) {
        Vertex vertex;  
        uint64_t curr_id = 0; 
        while(1) {
            std::ifstream ifs(traj_space_path_ + "/Vertex/id_" + std::to_string(curr_id));
            if(!ifs) {
                break;
            }
            curr_id++;  
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Session Edge object  获取某一个轨迹的所有边   
     * 
     * @param session 
     * @param session_vertexs 
     */
    void GetTrajectoryEdge(const uint16_t& traj, std::deque<Edge>& traj_edges) {
        Edge edge;  
        uint64_t curr_id = 0; 
        while(1) {
            std::ifstream ifs(traj_space_path_ + "/Edge/id_" + std::to_string(curr_id));
            if(!ifs) {
                break;
            }
            curr_id++;  
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Trajectory Edge object
     * 
     * @param traj 
     * @return const std::deque<Edge>& 
     */
    const auto& GetTrajectoryEdge(const uint16_t& traj) {
        return traj_edge_map_[traj];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the All Edge object
     * @return std::deque<Edge> const& 
     */
    std::deque<Edge> const& GetAllEdge() {
        return edge_container_;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        /**
         * @todo
         */
        return true;  
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        std::string file_path = traj_space_path_ + "/KeyFramePoints/key_frame_" 
                + name + std::to_string(global_id) + ".pcd";

        if (pcl::io::loadPCDFile(file_path, *curr_points) < 0) {
            return false;
        }

        return true;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Key Frame Point Cloud object
     * 
     * @tparam _PointT 
     * @param name 
     * @param global_id 
     * @param curr_points 
     * @return true 
     * @return false 
     */
    template<typename _PointT>
    bool GetKeyFramePointCloud(std::string const& name, uint32_t const& global_id, 
            pcl::PointCloud<_PointT>& curr_points) {
        curr_points.clear();  
        std::string file_path = traj_space_path_ + "/KeyFramePoints/key_frame_" 
                + name + std::to_string(global_id) + ".pcd";

        if (pcl::io::loadPCDFile(file_path, curr_points) < 0) {
            return false;
        }

        return true;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the DataBase Info object
     * 
     * @return const Info& 
     */
    const Info& GetDataBaseInfo() const {
        return info_;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Get the Trajectory I D List object
     */
    std::vector<uint16_t> GetTrajectoryIDList() {
        std::vector<uint16_t> id_list;

        for (auto& item : traj_vertex_map_) {
            id_list.push_back(item.first);
        }

        return id_list;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 根据id获取vertex的pose 
     * @param id
     */            
    inline bool SearchVertexPose(uint64_t const& id, Eigen::Isometry3d &pose) const {
        Vertex v = GetVertexByID(id); 
        pose = v.pose_;
        return true;  
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
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
        // 删除target
        traj_vertex_map_.erase(target_traj); 
        traj_edge_map_.erase(target_traj); 
        traj_keyframePositionCloud_.erase(target_traj);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 
     * 
     * @param traj 
     * @param index 
     * @param pose 
     */
    void UpdataKeyframePointcloud(const uint16_t& traj, const uint32_t& index, 
                                                                            const Eigen::Isometry3d& pose) {
        // std::cout << "UpdataKeyframePointcloud" << std::endl;
        auto& keyframePositionCloud = traj_keyframePositionCloud_.at(traj);
        (*keyframePositionCloud)[index].x = pose.translation().x();
        (*keyframePositionCloud)[index].y = pose.translation().y();
        (*keyframePositionCloud)[index].z = pose.translation().z();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief 
     * 
     */
    void DataReset() {
        cloudKeyFramePosition3D_->clear();
        cloudKeyFrameRot3D_->clear();
        database_vertex_info_.clear();
        traj_keyframePositionCloud_.clear();
        traj_vertex_map_.clear();
        traj_edge_map_.clear(); 
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
        traj_space_path_ = ""; 
    }

    PoseGraphDataBase(PoseGraphDataBase const& object) {}
    PoseGraphDataBase(PoseGraphDataBase&& object) {}
private:
    Info info_;
    bool has_new_keyframe_ = false; 
    std::string traj_space_path_;  
    std::string session_path_; 
    boost::shared_mutex pose_cloud_mt_, database_mt_, loop_mt_;  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudKeyFramePosition3D_;  // 历史关键帧位置
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudKeyFrameRot3D_; // 历史关键帧姿态   四元数形式  
    std::unordered_map<uint16_t, pcl::PointCloud<pcl::PointXYZ>::Ptr> traj_keyframePositionCloud_;
    // 按照id先后顺序排列   <轨迹id，该vertex在轨迹中的局部index>  
    std::vector<std::pair<uint16_t, uint32_t>> database_vertex_info_; 
    // <轨迹id，Vertex的集合(排列顺序按照id的从小到大，即先创建的Vertex在前，后创建的在后)>    
    std::unordered_map<uint16_t, std::vector<Vertex>> traj_vertex_map_;
    // <轨迹id，整个轨迹所有edge的集合(按创建时间先后顺序)>
    std::unordered_map<uint16_t, std::vector<Edge>> traj_edge_map_;
    std::deque<Edge> edge_container_; // 保存图的边
    std::deque<Vertex> vertex_container_;   // 保存图节点
    /**
     * @todo 尝试下面的数据结构  
     * 
     */
    std::unordered_map<uint16_t, std::list<uint32_t>> traj_vertex_list_;
    std::unordered_map<uint16_t, std::list<Edge>> traj_edge_list_;
}; // class 
} // namespace 

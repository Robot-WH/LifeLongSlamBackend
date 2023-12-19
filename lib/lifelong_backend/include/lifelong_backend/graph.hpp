/**
 * @file graph.hpp
 * @brief 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2023 
 * 
 */
#pragma once
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
namespace lifelong_backend {

/**
 * @brief: pose-graph 顶点
 */
struct Vertex {
    Vertex() : id_(0), traj_(0), pose_(Eigen::Isometry3d::Identity()) {}
    Vertex(uint64_t const& id, uint16_t const& traj, Eigen::Isometry3d const& pose) 
        : id_(id), traj_(traj), pose_(pose) {}
    
    void SetPose(Eigen::Isometry3d const& pose) {
        pose_ = pose;  
    }
    
    /**
     * @brief 
     * 
     * @param path 
     */
    void Save(std::string const& path) {
        if (!boost::filesystem::is_directory(path)) {
            boost::filesystem::create_directory(path);
        }

        std::ofstream ofs(path + "/Vertex/id_" + std::to_string(id_));
        ofs << "traj " << traj_ << "\n";  
        ofs << "id " << id_ << "\n";
        ofs << "pose\n";
        ofs << pose_.matrix() <<"\n";
    }

    uint16_t traj_;     // 轨迹id    支持 multi - session
    uint64_t id_;      // 全局id
    Eigen::Isometry3d pose_;
}; 

/**
 * @brief: pose-graph 边
 * @details: 边连接的节点idx，约束 + 协方差  
 */
struct Edge {
    // 边类型 
    Edge() {}
    Edge(int16_t const& traj, uint64_t id, uint64_t head_id, uint64_t tail_id, 
                Eigen::Isometry3d const& constraint, Eigen::Matrix<double, 1, 6> const& noise) 
                : traj_(traj), id_(id), link_id_(head_id, tail_id), constraint_(constraint), noise_(noise) {}

    /**
     * @brief 
     * 
     * @param path 
     */
    void Save(std::string const& path) {
        if (!boost::filesystem::is_directory(path)) {
          boost::filesystem::create_directory(path);
        }

        std::ofstream ofs(path + "/Edge/id_" + std::to_string(id_));
        ofs << "traj " << traj_ << "\n";
        ofs << "id " << id_ << "\n";
        ofs << "link_head\n";
        ofs << link_id_.first <<"\n";
        ofs << "link_tail\n";
        ofs << link_id_.second <<"\n";
        ofs << "constraint\n";
        ofs << constraint_.matrix()<<"\n";
        ofs << "noise\n";
        ofs << noise_.matrix()<<"\n";
    }

    int16_t traj_ = -1;     // 该edge所属的轨迹
    uint64_t id_ = 0;   // 该边的全局id  
    std::pair<uint64_t, uint64_t> link_id_{-1, -1};     // 该边 连接的 节点 全局id 
    Eigen::Isometry3d constraint_ = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 1, 6> noise_ = Eigen::Matrix<double, 1, 6>::Zero();     // 6 dof 约束的 噪声 向量  xyz  + rpy
}; 

/**
 * @brief 回环边
 *  包含session信息
 */
struct LoopEdge : public Edge {
    LoopEdge() {}
    LoopEdge(int16_t traj, uint64_t id, int16_t loop_traj, uint64_t head_id, uint64_t tail_id, 
    Eigen::Isometry3d const& constraint, Eigen::Matrix<double, 1, 6> const& noise) 
        : Edge(traj, id, head_id, tail_id, constraint, noise), loop_traj_(loop_traj) {}
    int16_t loop_traj_ = -1;   // 闭环的轨迹的id   
};
}
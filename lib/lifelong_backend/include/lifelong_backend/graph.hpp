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
#include "graph.pb.h"
namespace lifelong_backend {

static lifelong_backend::transform::proto::Vector3d ToProto(const Eigen::Vector3d& vector) {
    lifelong_backend::transform::proto::Vector3d v;
    v.set_x(vector.x());
    v.set_y(vector.y());
    v.set_z(vector.z());
    return v;
}

static lifelong_backend::transform::proto::Quaterniond ToProto(const Eigen::Quaterniond& q) {
    lifelong_backend::transform::proto::Quaterniond proto_q;
    proto_q.set_x(q.x());
    proto_q.set_y(q.y());
    proto_q.set_z(q.z());
    proto_q.set_w(q.w());
    return proto_q;
}

static Eigen::Vector3d ProtoTo(const lifelong_backend::transform::proto::Vector3d& proto_vector) {
    return Eigen::Vector3d(proto_vector.x(), proto_vector.y(), proto_vector.z());
}

static Eigen::Quaterniond ProtoTo(const lifelong_backend::transform::proto::Quaterniond& proto_q) {
    return Eigen::Quaterniond(proto_q.w(), proto_q.x(), proto_q.y(), proto_q.z());
}

static Eigen::Isometry3d ProtoTo(const lifelong_backend::transform::proto::Transform3d& transform) {
    Eigen::Isometry3d T;
    T.translation() = ProtoTo(transform.translation());
    T.linear() = ProtoTo(transform.rotation()).toRotationMatrix();
    return T;
}

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
            // std::cout << "Save, path: " << path << std::endl;
            boost::filesystem::create_directory(path);
        }
        // std::ofstream ofs(path + "/Vertex/id_" + std::to_string(id_));
        // ofs << "traj " << traj_ << "\n";  
        // ofs << "id " << id_ << "\n";
        // ofs << "pose\n";
        // ofs << pose_.matrix() <<"\n";
        std::fstream ofs(path + "/Vertex/id_" + std::to_string(id_), 
            std::ios::out | std::ios::trunc | std::ios::binary);

        lifelong_backend::graph::proto::Vertex vertex;
        vertex.set_traj(traj_);
        vertex.set_id(id_);
        // lifelong_backend::transform::proto::Transform3d* pose(new lifelong_backend::transform::proto::Transform3d);
        // lifelong_backend::transform::proto::Vector3d* translation(new lifelong_backend::transform::proto::Vector3d);
        // lifelong_backend::transform::proto::Quaterniond* Quaterniond(new lifelong_backend::transform::proto::Quaterniond);
        // *translation = ToProto(pose_.translation());
        // *Quaterniond = ToProto(Eigen::Quaterniond(pose_.rotation()));
        // pose->set_allocated_translation(translation);
        // pose->set_allocated_rotation(Quaterniond);
        // vertex.set_allocated_pose(pose);
        const auto& m_pose = pose_.matrix();  
        vertex.add_pose(m_pose(0, 0)); 
        vertex.add_pose(m_pose(0, 1)); 
        vertex.add_pose(m_pose(0, 2)); 
        vertex.add_pose(m_pose(0, 3)); 
        vertex.add_pose(m_pose(1, 0)); 
        vertex.add_pose(m_pose(1, 1)); 
        vertex.add_pose(m_pose(1, 2)); 
        vertex.add_pose(m_pose(1, 3)); 
        vertex.add_pose(m_pose(2, 0)); 
        vertex.add_pose(m_pose(2, 1)); 
        vertex.add_pose(m_pose(2, 2)); 
        vertex.add_pose(m_pose(2, 3)); 
        vertex.add_pose(m_pose(3, 0)); 
        vertex.add_pose(m_pose(3, 1)); 
        vertex.add_pose(m_pose(3, 2)); 
        vertex.add_pose(m_pose(3, 3)); 

        if (!vertex.SerializeToOstream(&ofs)) {
            // cerr << "Failed to write vertex." << endl;
            return;
        }
    }

    /**
     * @brief 
     * 
     * @param path 
     * @return true 
     * @return false 
     */
    bool Load(const std::string& path) {
        // std::ifstream ifs(path);
        std::fstream ifs(path, std::ios::in | std::ios::binary);

        if(!ifs) {
            return false;
        }

        lifelong_backend::graph::proto::Vertex vertex;

        if (!vertex.ParseFromIstream(&ifs)) {
            // cerr << "Failed to parse address book." << endl;
            return false;
        }

        traj_ = vertex.traj();
        id_ = vertex.id();
        auto& m_pose = pose_.matrix();
        m_pose(0, 0) = vertex.pose(0);
        m_pose(0, 1) = vertex.pose(1);
        m_pose(0, 2) = vertex.pose(2);
        m_pose(0, 3) = vertex.pose(3);
        m_pose(1, 0) = vertex.pose(4);
        m_pose(1, 1) = vertex.pose(5);
        m_pose(1, 2) = vertex.pose(6);
        m_pose(1, 3) = vertex.pose(7);
        m_pose(2, 0) = vertex.pose(8);
        m_pose(2, 1) = vertex.pose(9);
        m_pose(2, 2) = vertex.pose(10);
        m_pose(2, 3) = vertex.pose(11);
        m_pose(3, 0) = vertex.pose(12);
        m_pose(3, 1) = vertex.pose(13);
        m_pose(3, 2) = vertex.pose(14);
        m_pose(3, 3) = vertex.pose(15);

        // pose_ = ProtoTo(vertex.pose()); 
        return true;  
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
    Edge(const int16_t& traj, const uint64_t& id, const uint64_t& head_id, 
                const uint64_t& tail_id, const uint32_t& link_head_local_index,
                const Eigen::Isometry3d& constraint, const Eigen::Matrix<double, 1, 6>& noise) 
                : traj_(traj), id_(id), link_id_(head_id, tail_id), link_head_local_index_(link_head_local_index), 
                    constraint_(constraint), noise_(noise) {}

    /**
     * @brief 
     * 
     * @param path 
     */
    void Save(std::string const& path) {
        if (!boost::filesystem::is_directory(path)) {
          boost::filesystem::create_directory(path);
        }

        // std::ofstream ofs(path + "/Edge/id_" + std::to_string(id_));
        // ofs << "traj " << traj_ << "\n";
        // ofs << "id " << id_ << "\n";
        // ofs << "link_head " << link_id_.first << "\n";
        // ofs << "link_tail " << link_id_.second << "\n";
        // ofs << "constraint\n";
        // ofs << constraint_.matrix()<<"\n";
        // ofs << "noise\n";
        // ofs << noise_.matrix()<<"\n";

        std::fstream ofs(path + "/Edge/id_" + std::to_string(id_), 
            std::ios::out | std::ios::trunc | std::ios::binary);
        
        lifelong_backend::graph::proto::Edge edge;
        edge.set_traj(traj_);
        edge.set_id(id_);
        edge.set_link_head(link_id_.first); 
        edge.set_link_tail(link_id_.second);   

        lifelong_backend::transform::proto::Transform3d* constraint(new lifelong_backend::transform::proto::Transform3d);
        lifelong_backend::transform::proto::Vector3d* translation(new lifelong_backend::transform::proto::Vector3d);
        lifelong_backend::transform::proto::Quaterniond* Quaterniond(new lifelong_backend::transform::proto::Quaterniond);
        *translation = ToProto(constraint_.translation());
        *Quaterniond = ToProto(Eigen::Quaterniond(constraint_.rotation()));
        constraint->set_allocated_translation(translation);
        constraint->set_allocated_rotation(Quaterniond);
        edge.set_allocated_constraint(constraint);

        edge.add_noise(noise_(0, 0)); 
        edge.add_noise(noise_(0, 1)); 
        edge.add_noise(noise_(0, 2)); 
        edge.add_noise(noise_(0, 3)); 
        edge.add_noise(noise_(0, 4)); 
        edge.add_noise(noise_(0, 5)); 

        if (!edge.SerializeToOstream(&ofs)) {
            // cerr << "Failed to write vertex." << endl;
            // return;
        }
    }

    /**
     * @brief 
     * 
     * @param path 
     * @return true 
     * @return false 
     */
    bool Load(const std::string& path) {
        // std::ifstream ifs(path);
        std::fstream ifs(path, std::ios::in | std::ios::binary);

        if(!ifs) {
            return false;
        }

        lifelong_backend::graph::proto::Edge edge;

        if (!edge.ParseFromIstream(&ifs)) {
            // cerr << "Failed to parse address book." << endl;
            return false;
        }

        traj_ = edge.traj();
        id_ = edge.id();
        link_id_.first = edge.link_head();
        link_id_.second = edge.link_tail();
        constraint_ = ProtoTo(edge.constraint()); 
        noise_(0, 0) = edge.noise(0);
        noise_(0, 1) = edge.noise(1);
        noise_(0, 2) = edge.noise(2);
        noise_(0, 3) = edge.noise(3);
        noise_(0, 4) = edge.noise(4);
        noise_(0, 5) = edge.noise(5);

        return true;  
    }

    int16_t traj_ = -1;     // 该edge所属的轨迹
    uint64_t id_ = 0;   // 该边的全局id  
    uint32_t link_head_local_index_ = 0;    // 连接头结点在位姿图中的局部index    
    std::pair<uint64_t, uint64_t> link_id_{0, 0};     // 该边 连接的 节点 全局id 
    Eigen::Isometry3d constraint_ = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 1, 6> noise_ = Eigen::Matrix<double, 1, 6>::Zero();     // 6 dof 约束的 噪声 向量  xyz  + rpy
}; 

/**
 * @brief 回环边
 *  包含session信息
 */
struct LoopEdge : public Edge {
    LoopEdge() {}
    // LoopEdge(int16_t traj, int16_t loop_traj, uint64_t id, uint64_t head_id, uint64_t tail_id, 
    //         const uint32_t& link_head_local_index, Eigen::Isometry3d const& constraint, 
    //         Eigen::Matrix<double, 1, 6> const& noise) 
    //             : Edge(traj, id, head_id, tail_id, link_head_local_index, constraint, noise), loop_traj_(loop_traj) {}
    int16_t loop_traj_ = -1;   // 闭环的轨迹的id   
    Eigen::Isometry3d loop_vertex_pose_; 
};
}
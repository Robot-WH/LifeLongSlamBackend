/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-29 12:48:41
 * @Description: 
 * @Others: 
 */
#pragma once 
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "graph_optimization.h"
namespace lifelong_backend {
    class GtsamGraphOptimizer: public GraphOptimizerInterface {
public:
    GtsamGraphOptimizer();
    void Rebuild(std::vector<Vertex> const& vertexs, std::vector<Edge> const& edges) override; 
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 
     * @details: 
     * @param flag 0 表示普通优化   1表示回环优化 
     */    
    bool Optimize(uint8_t flag = 0) override;

    // 输出数据
    bool GetAllGraphNodePose(std::deque<Eigen::Matrix4f>& optimized_pose) override;
    Eigen::Isometry3d GetNodePose(uint64_t const& id) override;
    void SetNodePose(uint64_t const& id, Eigen::Isometry3d const& pose) override; 
    uint64_t GetNodeNum() override;
    // 添加节点、边、鲁棒核
    void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) override;
    void AddSe3Node(const Eigen::Isometry3d &pose, uint64_t const& id, bool need_fix = false) override;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 添加se3 节点间的约束
     * @details: 
     * @param relative_pose T vertex_index1<-vertex_index2           poseindex1.inverse() * poseindex2
     * @return {*}
     */   
    void AddSe3Edge(uint64_t vertex_index1,
                    uint64_t vertex_index2,
                    const Eigen::Isometry3d &relative_pose,
                    const Eigen::VectorXd noise) override;
    void AddSe3PriorXYZEdge(uint64_t se3_vertex_index,
                            const Eigen::Vector3d &xyz,
                            Eigen::VectorXd noise) override;
    void AddSe3PriorQuaternionEdge(uint64_t se3_vertex_index,
                                const Eigen::Quaterniond &quat,
                                Eigen::VectorXd noise) override;
private:
    gtsam::Pose3 trans2gtsamPose(Eigen::Isometry3d const& pose);

    // gtsam
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;

    std::string robust_kernel_name_;
    double robust_kernel_size_;
    bool need_robust_kernel_ = false;
    Eigen::VectorXd prior_noise_;  // 先验噪声  
    uint64_t node_num_ = 0;  

}; // class 
} // namespace Slam3D

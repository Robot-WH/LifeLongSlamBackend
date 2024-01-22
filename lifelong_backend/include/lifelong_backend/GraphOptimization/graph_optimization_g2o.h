/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-29 12:48:25
 * @Description: 
 * @Others: 
 */

#pragma once 

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>



#include "graph_optimization.h"

namespace g2o {
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
} // namespace g2o

G2O_USE_TYPE_GROUP(slam3d);

// LinearSolverDense  使用dense cholesky分解法 
// LinearSolverEigen 稀疏cholesky法   效果和CSparse相似    
// 因为SLAM的优化问题一般都具有稀疏性，所以一般不用Dense方法
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)
G2O_USE_OPTIMIZATION_LIBRARY(eigen)
// namespace g2o {
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
// } // namespace g2o

// LinearSolverEigen:
// 基于Eigen库的线性求解器。
// 使用Eigen库中的矩阵和线性代数运算。
// 适用于小到中等规模的优化问题，特别是当问题的稀疏性不太重要时。
// 对于大规模问题，可能会受限于内存消耗和计算效率。

// LinearSolverCSparse:
// 基于CSparse库的线性求解器。
// 专门针对稀疏矩阵的优化问题，适用于大规模优化问题。
// 通常用于解决需要大量稀疏矩阵操作的SLAM和图优化问题。
// 由于其高效的内存和计算效率，通常在大规模场景中表现良好。

// LinearSolverCholmod:
// 基于Cholmod库的线性求解器。
// 主要用于处理稀疏正定对称矩阵。
// 在某些图优化问题中，特别是需要求解大规模正定对称矩阵的情况下，表现良好。
// 适用于一些SLAM系统和非线性优化问题，通常需要良好的数值稳定性。

// LinearSolverPCG (Preconditioned Conjugate Gradient):
// 基于共轭梯度方法的线性求解器。
// 适用于一些特殊类型的线性系统，特别是对称正定系统。
// 可以配合预处理方法一起使用，以提高求解效率和数值稳定性。
// 在特定问题和配置下，可以表现出良好的性能。

namespace lifelong_backend {
    
class G2oGraphOptimizer: public GraphOptimizerInterface {
public:
    G2oGraphOptimizer(const std::string &solver_type = "lm_var");

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 根据节点信息和边的信息重建位姿图   
     */    
    void Rebuild(std::vector<Vertex> const& vertexs, std::vector<Edge> const& edges) override; 
    // 优化
    bool Optimize(uint8_t flag = 0) override;
    void Reset() override;  
    // 输出数据
    bool GetAllGraphNodePose(std::deque<Eigen::Matrix4f>& optimized_pose) override;
    Eigen::Isometry3d GetNodePose(uint64_t const& id) override;
    void SetNodePose(uint64_t const& id, Eigen::Isometry3d const& pose) override; 
    uint64_t GetNodeNum() override;
    // 添加节点、边、鲁棒核
    void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) override;
    void AddSe3Node(const Eigen::Isometry3d &pose, uint64_t const& id, bool need_fix = false) override;
    void AddSe3Edge(uint64_t vertex_index1,
                    uint64_t vertex_index2,
                    const Eigen::Isometry3d &relative_pose,
                    const Eigen::VectorXd noise) override;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 位置先验约束 
     * @details 比如GPS
     * @param se3_vertex_index 添加约束的节点index 
     */    
    void AddSe3PriorXYZEdge(uint64_t se3_vertex_index,
                            const Eigen::Vector3d &xyz,
                            Eigen::VectorXd noise) override;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 姿态先验约束 
     * @details 比如惯导的姿态观测，地面的观测
     * @param se3_vertex_index 添加约束的节点index 
     */    
    void AddSe3PriorQuaternionEdge(uint64_t se3_vertex_index,
                                const Eigen::Quaterniond &quat,
                                Eigen::VectorXd noise) override;

private:
    Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);
    Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise);
    Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);
    void AddRobustKernel(g2o::OptimizableGraph::Edge *edge, const std::string &kernel_type, double kernel_size);

private:
    g2o::RobustKernelFactory *robust_kernel_factory_;
    std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;

    std::string robust_kernel_name_;
    double robust_kernel_size_;
    bool need_robust_kernel_ = false;
}; // class 
} // namespace Slam3D
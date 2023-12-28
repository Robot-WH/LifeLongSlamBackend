/**
 * @file graph_optimization.h
 * @brief 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2022 
 * 
 */
#pragma once 

#include <string>
#include <deque>
#include <Eigen/Dense>
#include "../graph.hpp"

namespace lifelong_backend {
class GraphOptimizerInterface {
public:
  virtual ~GraphOptimizerInterface() {}
  virtual void Rebuild(std::vector<Vertex> const& vertexs, std::vector<Edge> const& edges) = 0; 
  // 优化
  virtual bool Optimize(uint8_t flag = 0) = 0;
  virtual void Reset() {}
  // 输入、输出数据
  virtual bool GetAllGraphNodePose(std::deque<Eigen::Matrix4f>& optimized_pose) = 0;
  virtual Eigen::Isometry3d GetNodePose(uint64_t const& id) = 0;
  virtual uint64_t GetNodeNum() = 0;
  virtual void SetNodePose(uint64_t const& id, Eigen::Isometry3d const& pose) = 0;
  // 添加节点、边、鲁棒核
  virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) = 0;
  virtual void AddSe3Node(const Eigen::Isometry3d &pose, uint64_t const& id, bool need_fix = false) = 0;
  virtual void AddSe3Edge(uint64_t vertex_index1,
                                                      uint64_t vertex_index2,
                                                      const Eigen::Isometry3d &relative_pose,
                                                      const Eigen::VectorXd noise) = 0;
  virtual void AddSe3PriorXYZEdge(uint64_t se3_vertex_index,
                                  const Eigen::Vector3d &xyz,
                                  Eigen::VectorXd noise) = 0;
  virtual void AddSe3PriorQuaternionEdge(uint64_t se3_vertex_index,
                                          const Eigen::Quaterniond &quat,
                                          Eigen::VectorXd noise) = 0;
  // 设置优化参数
  void SetMaxIterationsNum(uint16_t max_iterations_num);

protected:
  uint16_t max_iterations_num_ = 512;
};
} // namespace Slam3D

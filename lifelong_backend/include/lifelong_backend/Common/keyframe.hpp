/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2021-12-18 18:21:53
 * @Description: 
 * @Others: 
 */
#pragma once 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Dense>
namespace lifelong_backend {
/**
 * @brief KeyFrame  主要保存关键帧的各种观测信息
 */
class KeyFrame {
public:
  using Ptr = std::shared_ptr<KeyFrame>;
  using ConstPtr = std::shared_ptr<const KeyFrame>; 
  KeyFrame() {}
  // 关键帧不储存点云数据  
  KeyFrame(double const& stamp, Eigen::Isometry3d const& odom, int64_t id = -1)
  : time_stamp_(stamp), odom_(odom), id_(id) {}

  virtual ~KeyFrame() {}

  // 关键帧的数据结构
  double time_stamp_ = 0.0;                                // timestamp
  int64_t id_ = -1;   // 对应点云文件  在文件夹中的标识    以及   节点
  int64_t adjacent_id_ = -1; // 上一个连接的关键帧 id 
  Eigen::Isometry3d odom_ = Eigen::Isometry3d::Identity();     // odometry (estimated by scan_matching_odometry)   
  Eigen::Isometry3d between_constraint_ = Eigen::Isometry3d::Identity();  // 与上一个关键帧的相对约束   
  boost::optional<Eigen::Vector4d> floor_coeffs_;   // detected floor's coefficients    地面信息 
  boost::optional<Eigen::Vector3d> utm_coord_;                       // UTM coord obtained by GPS    UTM坐标   
  boost::optional<Eigen::Vector3d> acceleration_;                       // UTM coord obtained by GPS    UTM坐标   
  boost::optional<Eigen::Quaterniond> orientation_;               // imu测得的激光姿态
  bool gnss_matched_ = false;  
  bool GNSS_valid_ = false;  // 是否使用GNSS 观测
  bool IMU_valid_ = false;     // 是否使用IMU姿态观测 
  bool planeConstraint_valid_ = false;  // 是否使用 平面约束观测 
};
}



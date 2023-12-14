/**
 * @file pose_graph_database.hpp
 * @brief   保存位姿图的数据库管理模块 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2022
 * 
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
#include "Common/keyframe.hpp"
#include "graph.hpp"
namespace lifelong_backend {
/**
 * @brief: 维护 pose graph 的数据 
 */    
class PoseGraphDataBase {
private:
    struct Info {
        uint32_t session_num;  
        uint64_t keyframe_num;  
        uint32_t edge_num; 
    };
public:
    
    /**
     * @brief: 单例的创建函数  
     */            
    static PoseGraphDataBase& GetInstance() {
        static PoseGraphDataBase PoseGraph_dataBase; 
        return PoseGraph_dataBase; 
    }

    
    /**
     * @brief: 保存数据
     * @details pose-graph的数据 保存在 session 中  
     */            
    void Save() {   
        // 保存keyframe信息 
        assert(database_save_path_ != ""); 
        // 保存pose-graph
        // 节点
        for (auto& item : vertex_container_) {
            item.Save(database_save_path_); 
        }
        // for (uint64_t i = 0; i < vertex_container_.size(); i++) {
        //     vertex_container_[i].Save(database_save_path_);  
        // }
        // 边
        for (uint64_t i = 0; i < edge_container_.size(); i++) {
            edge_container_[i].Save(database_save_path_);  
        }
        // // 保存位姿点云
        // std::string file_path = database_save_path_ + "/Position3D.pcd";
        // pcl::io::savePCDFileBinary(file_path, *cloudKeyFramePosition3D_);
        // file_path = database_save_path_ + "/Rot3D.pcd";
        // pcl::io::savePCDFileBinary(file_path, *cloudKeyFrameRot3D_);
        // 更新info文件
        std::ofstream ofs(database_save_path_ + "/info");
        ofs << "session_num " << info_.session_num + 1 << "\n";
        ofs << "keyframe_num " << vertex_container_.back().id_ + 1 << "\n";
        ofs << "edge_num " << info_.edge_num << "\n";
    }
    
    
    /**
     * @brief: 从指定路径中加载数据库  
     */            
    bool Load(std::string database_save_path) {
        database_save_path_ = database_save_path;  
        assert(database_save_path_ != ""); 
        // 不管是不是第一次session，都默认以session_0为main session 
        // session_path_ = database_save_path_ + "/session_0"; 
        // 读取当前area的Info
        std::ifstream ifs(database_save_path_ + "/info");
            
        if(!ifs) {
            std::cout << "开启第一个session..." << std::endl;
            info_.session_num = 0;  
            info_.keyframe_num = 0;  
            info_.edge_num = 0;  
            // 创建文件夹 session_0
            boost::filesystem::create_directory(database_save_path_ + "/KeyFrameDescriptor");
            boost::filesystem::create_directory(database_save_path_ + "/KeyFramePoints");
            boost::filesystem::create_directory(database_save_path_ + "/Vertex");
            boost::filesystem::create_directory(database_save_path_ + "/Edge");
            return false;
        } 

        std::cout << SlamLib::color::GREEN << "加载当前Area的Info..." << std::endl;

        while(!ifs.eof()) {
            std::string token;
            ifs >> token;

            if (token == "session_num") {
                ifs >> info_.session_num; 
                std::cout << "session数量: " << info_.session_num <<std::endl;
            } else if (token == "keyframe_num") {
                ifs >> info_.keyframe_num; 
                std::cout << "keyframe总数: " << info_.keyframe_num <<std::endl;
            } else if (token == "edge_num") {
                ifs >> info_.edge_num; 
                std::cout << "edge总数: " << info_.edge_num << SlamLib::color::RESET <<std::endl;
            }
        }

        if (!LoadPoseGraph()) {
            DataReset(); 
            LOG(INFO) << SlamLib::color::RED<<"Load Pose-Graph failure" 
                << SlamLib::color::RESET << std::endl;
            return false; 
        }

        LOG(INFO) << SlamLib::color::GREEN<<"Load Pose-Graph done" 
            << SlamLib::color::RESET << std::endl;

        // // 加载姿态点云
        // std::string file_path = database_save_path_ +"/Position3D.pcd";
        
        // if (pcl::io::loadPCDFile(file_path, *cloudKeyFramePosition3D_) < 0) {
        //     LOG(INFO) << SlamLib::color::RED << "Load Position3D error !";
        //     DataReset(); 
        //     return false; 
        // }

        // file_path = database_save_path_ +"/Rot3D.pcd";

        // if (pcl::io::loadPCDFile(file_path, *cloudKeyFrameRot3D_) < 0) {
        //     LOG(INFO) << SlamLib::color::RED << "Load Rot3D error !";
        //     DataReset();
        //     return false;
        // } 

        return true; 
    }

    
    /**
     * @brief 
     * @return true 
     * @return false 
     */
    // bool LoadKeyFrame() {
    //     uint64_t index = 0; 
    //     // 加载keyframe数据
    //     while(1) {
    //         KeyFrame keyframe;  
    //         std::ifstream ifs(database_save_path_ + "/KeyFrameData/data_" + std::to_string(index));
            
    //         if(!ifs) {
    //             break;
    //         }

    //         index++;  

    //         while(!ifs.eof()) {
    //             std::string token;
    //             ifs >> token;

    //             if(token == "stamp") {
    //                 ifs >> keyframe.time_stamp_; 
    //             } else if (token == "id") {
    //                 ifs >> keyframe.id_; 
    //             } else if (token == "odom") {
    //                 Eigen::Matrix4d matrix; 
    //                 for(int i = 0; i < 4; i++) {
    //                     for(int j = 0; j < 4; j++) {
    //                         ifs >> matrix(i, j);
    //                     }
    //                 }
    //                 keyframe.odom_.translation() = matrix.block<3, 1>(0, 3);
    //                 keyframe.odom_.linear() = matrix.block<3, 3>(0, 0);
    //             }
    //             else if (token == "utm_coord") {
    //             }
    //             else if (token == "floor_coeffs") {
    //             }
    //             else if (token == "acceleration") {
    //             }
    //             else if (token == "orientation") {
    //             }
    //         }

    //         keyframe_database_.push_back(std::move(keyframe));
    //     }

    //     if (index == 0) {
    //         return false;
    //     }

    //     LOG(INFO) << SlamLib::color::GREEN << "Load KetFrame done, num: " <<
    //         keyframe_database_.size() << SlamLib::color::RESET;
    //     return true; 
    // }

    
    /**
     * @brief 
     * @return true 
     * @return false 
     */
    bool LoadPoseGraph() {
        uint64_t index = 0; 
        // 加载vertex数据
        while(1) {
            Vertex vertex;  
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
                } else if (token == "session") {
                    ifs >> vertex.session_; 
                }
            }

            vertex_container_.emplace_back(std::move(vertex));
            AddPosePoint(vertex.pose_);
        }
        
        if (vertex_container_.size() == 0) {
            // std::cout<<common::RED<<
            // "Load KetFrame ERROR: vertex num keyframe num not match,  vertex num: "
            // <<vertex_container_.size()<<", keyframe num: "<<
            // keyframe_database_.size()<<std::endl;
            return false;  
        }

        index = 0;  
        // 加载edge数据
        while(1) {
            Edge edge;  
            std::ifstream ifs(database_save_path_ + "/Edge/id_" + std::to_string(index));
            
            if(!ifs) {
                break;
            }

            index++;  

            while(!ifs.eof()) {
                std::string token;
                ifs >> token;

                if (token == "id") {
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

            edge_container_.push_back(std::move(edge));
        }

        return true; 
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
    void AddKeyFrameData(KeyFrame const& keyframe) {
        // 将关键帧数据保存 
        database_mt_.lock();  
        keyframe_database_.push_back(keyframe);  
        database_mt_.unlock();  
        has_new_keyframe_ = true; 
    }

    
    /**
     * @brief 添加一个位姿图节点
     * @param  id               My Param doc
     * @param  pose             My Param doc
     */
    void AddVertex(uint64_t const& id, uint32_t const& session, Eigen::Isometry3d const& pose) {
        // vertex_container_.insert(std::pair(id, Vertex(id, session, pose)));
        // vertex_container_[id] = Vertex(id, session, pose);
        global_to_local_ID_[id] = vertex_container_.size();  
        vertex_container_.emplace_back(id, session, pose);
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
     * @brief 添加一个位姿图边 
     * @param  head_id  首节点id
     * @param  tail_id    尾节点id
     * @param  constraint   边位姿约束
     * @param  noise    约束协方差
     */
    inline void AddEdge(int16_t const& traj, uint64_t const& head_id, uint64_t const& tail_id, 
            Eigen::Isometry3d const& constraint, Eigen::Matrix<double, 1, 6> const& noise) {
        edge_container_.emplace_back(traj, info_.edge_num, head_id, tail_id, constraint, noise); 
        ++info_.edge_num;
    }

    /**
     * @brief 重载
     * 
     * @param edge 
     */
    inline void AddEdge(Edge& edge) {
        edge.id_ = info_.edge_num; 
        edge_container_.push_back(edge); 
        ++info_.edge_num;
    }

    /**
     * @brief 读取当前posegraph中节点的数量 
     * 
     * @return uint64_t 
     */
    uint64_t ReadVertexNum() {
        database_mt_.lock_shared();
        uint64_t num = vertex_container_.size();
        database_mt_.unlock_shared();
        return num;  
    }

    /**
     * @brief 添加一个关键帧的点云数据 
     * 
     * @tparam _PointT 
     * @param name 
     * @param KF_index 
     * @param pointcloud 
     */
    template<typename _PointT>
    void AddKeyFramePointCloud(std::string const& name, 
                                                                    uint32_t const& KF_index, 
                                                                    pcl::PointCloud<_PointT> const& pointcloud) {
        std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
            + name + std::to_string(KF_index) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, pointcloud);
    }

    /**
     * @brief Get the Last Vertex object  
     * 
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
        if (global_to_local_ID_.find(id) != global_to_local_ID_.end()) {
            return vertex_container_[global_to_local_ID_.at(id)];
        }
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
            } else if (token == "session") {
                ifs >> vertex.session_; 
            }
        }

        return vertex;  
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
        if (!has_new_keyframe_) return false;  
        boost::shared_lock<boost::shared_mutex> lock(database_mt_); 
        keyframe = keyframe_database_.back();  
        has_new_keyframe_ = false; 
        return true;  
    }

    /**
     * @brief Get the Key Frame DataBase object
     * 
     * @return std::deque<KeyFrame> const& 
     */
    std::deque<KeyFrame> const& GetKeyFrameDataBase() {
        return keyframe_database_;
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
     * @brief Get the Session Vertex object
     * 
     * @param session 
     * @return std::deque<Vertex> const& 
     */
    void GetSessionVertex(uint16_t const& session, std::deque<Vertex>& session_vertexs) {
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
                } else if (token == "session") {
                    ifs >> vertex.session_; 
                }
            }
            if (vertex.session_ != session)  
                continue;  
            session_vertexs.emplace_back(std::move(vertex));
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
     * @brief Get the All Edge object
     * 
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
                                                                                typename pcl::PointCloud<_PointT>::ConstPtr &local_map) {
        pcl::PointCloud<_PointT> origin_points;   // 激光坐标系下的点云
        pcl::PointCloud<_PointT> trans_points;   // 转换到世界坐标系下的点云 
        typename pcl::PointCloud<_PointT>::Ptr map(new pcl::PointCloud<_PointT>()); 
        Vertex loop_vertex = GetVertexByID(center_id);
        std::cout << "loop_vertex session: " << loop_vertex.session_
            << ", id: " << loop_vertex.id_ << std::endl;

        for (int16_t i = -neighbors_num; i <= neighbors_num; i++ ) {
            int64_t curr_id = center_id + i;
            std::cout << "curr_id: " << curr_id << std::endl;
            Vertex curr_vertex = GetVertexByID(curr_id);
            // 处理边界
            if (curr_id < 0) {
                curr_id += 2 * neighbors_num + 1;
                std::cout << "curr_id < 0, adjust: " << curr_id << std::endl;
            }else if (curr_id >= info_.keyframe_num) {
                curr_id -= (2 * neighbors_num + 1); 
                std::cout << "curr_id >= info_.keyframe_num, adjust: " << curr_id << std::endl;
            } else if (loop_vertex.session_ != curr_vertex.session_) {
                // 进入不同的session了 
                if (i >= 0) {
                    curr_id -= (2 * neighbors_num + 1); 
                    std::cout << "loop_vertex.session_ != curr_vertex.session_, i >= 0, adjust: " << curr_id << std::endl;
                } else {
                    curr_id += (2 * neighbors_num + 1);
                    std::cout << "loop_vertex.session_ != curr_vertex.session_, i < 0, adjust: " << curr_id << std::endl;
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
     * @brief: 获取keyframe 的一个 名字为 name 的点云数据 
     * @param name 点云名称
     * @param id 关键帧在数据库中的id
     * @param[out] curr_points 读取的结果 
     * @return 
     */            
    template<typename _PointT>
    bool GetKeyFramePointCloud(std::string const& name, uint32_t const& id, 
            typename pcl::PointCloud<_PointT>::Ptr &curr_points) {
        std::string file_path = database_save_path_ + "/KeyFramePoints/key_frame_" 
                + name + std::to_string(id) + ".pcd";

        if (pcl::io::loadPCDFile(file_path, *curr_points) < 0) {
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
     * @brief 
     * 
     * @param id 
     * @param correct_pose 
     */
    inline void UpdateVertexPose(uint32_t id, Eigen::Isometry3d correct_pose) {
        boost::unique_lock<boost::shared_mutex> lock1(database_mt_);    // 写锁 
        boost::unique_lock<boost::shared_mutex> lock2(pose_cloud_mt_);    // 写锁 
        vertex_container_[id].SetPose(correct_pose);  // 节点 位姿更新  
        // 位姿点云更新   
        cloudKeyFramePosition3D_->points[id].x = correct_pose.translation().x();
        cloudKeyFramePosition3D_->points[id].y = correct_pose.translation().y();
        cloudKeyFramePosition3D_->points[id].y = correct_pose.translation().y();
        
        Eigen::Quaterniond q(correct_pose.rotation());
        cloudKeyFrameRot3D_->points[id].x = q.x();
        cloudKeyFrameRot3D_->points[id].y = q.y();
        cloudKeyFrameRot3D_->points[id].z = q.z();
        cloudKeyFrameRot3D_->points[id].intensity = q.w();   
    }

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

    /**
     * @todo  没什么用  准备删除 
     * 
     */
    std::deque<KeyFrame> keyframe_database_; // 保存全部关键帧的观测信息   


    std::deque<Edge> edge_container_; // 保存图节点
    std::deque<Vertex> vertex_container_; // 保存图的边
    std::unordered_map<uint64_t, uint64_t> global_to_local_ID_;  // 数据库id到位姿图id的映射  
}; // class 
} // namespace 
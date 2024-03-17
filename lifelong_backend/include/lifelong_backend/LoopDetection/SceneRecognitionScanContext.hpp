/**
 * @file SceneRecognitionScanContext.hpp
 * @brief 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#include "GlobalDescriptor/scanContext/Scancontext.hpp"
#include "SlamLib/Common/color.hpp"
#include "SlamLib/Common/pointcloud.h"
#include "../proto/sc.pb.h"
namespace lifelong_backend {
/**
 * @brief: 通过scan-context进行关键帧位置识别  
 */    
template<typename _PointCloudT>
class SceneRecognitionScanContext {
protected:
    using KeyVec = std::vector<std::vector<float>>;
    using ringKeyKdtree = KDTreeVectorOfVectorsAdaptor< KeyVec, float>;
    using FeatureContainer = SlamLib::FeaturePointCloudContainer<_PointCloudT>;
public:
    SceneRecognitionScanContext() {
        tree_making_period_conter_ = TREE_MAKING_PERIOD_ + 1; 
    }
    // SceneRecognitionScanContext(std::vector<std::string> target_names) 
    //     : target_names_(std::move(target_names)) {}
    virtual ~SceneRecognitionScanContext() {}

    /**
     * @brief: 对于一个关键帧点云计算 描述子保存 
     * @details: 
     * @param {PointCloud<_PointCloudT>} const
     * @return {*}
     */            
    uint32_t AddKeyFramePoints(FeatureContainer const& pcl_in) {
        pcl::PointCloud<_PointCloudT> selected_pointcloud; 
        extractInterestPointClouds(pcl_in, selected_pointcloud);

        Eigen::MatrixXd sc_desc = sc_.MakeScanContext(selected_pointcloud);                  // v1 提取sc全局特征描述符
        Eigen::MatrixXd ringkey = sc_.MakeRingkeyFromScanContext(sc_desc);   // 求 ring key 特征  
        
        std::lock_guard<std::mutex> lock(mt_);  
        polarcontexts_sc_.push_back(sc_desc);   // 当前帧点云检测完毕后   SC描述子 存放于 polarcontexts_sc_
        // 保存vector 类型的 ringkey  
        std::vector<float> polarcontext_ringkey = eig2vec(ringkey);
        polarcontext_ringkeys_vec_.push_back(polarcontext_ringkey);
        // 看看是否需要更新kdtree   
        if (tree_making_period_conter_ > TREE_MAKING_PERIOD_
                && polarcontext_ringkeys_vec_.size() > NUM_EXCLUDE_RECENT_) { 
            // to save computation cost    频率控制  
            // SlamLib::time::TicToc t_tree_construction;
            // std::vector<std::vector<float> > 类型
            polarcontext_ringkeys_to_search_.clear();
            // 构造用于建立kdtree的历史ringkey集合     
            // 这里减去 NUM_EXCLUDE_RECENT_ 也就是 不考虑最近的若干帧 
            polarcontext_ringkeys_to_search_.assign( // [first，end)
                polarcontext_ringkeys_vec_.begin(), 
                polarcontext_ringkeys_vec_.end() - NUM_EXCLUDE_RECENT_ );  // end() 指向 最后一个元素的后一个
            // KDTreeVectorOfVectorsAdaptor<>的 unique_ptr 
            polarcontext_ringkey_tree_.reset(
                new ringKeyKdtree(sc_.GetRingNum() ,// ring的维度 例如20     
                                                            polarcontext_ringkeys_to_search_, 
                                                            10 // 最后结点的最大叶子数   即kdtree最后一个结点包含元素的最大个数
                                                        )); 
            // t_tree_construction.toc("Tree construction");
            tree_making_period_conter_ = 0; 
        }

        tree_making_period_conter_++;
        return polarcontexts_sc_.size() - 1;
    }

    /**
     * @brief: 查找相似的点云
     * @details: 用于重定位 
     * @param scan_in 输入关键帧的所有点云特征数据
     * @return <id, 与相似帧的相对变换>
     */            
    std::pair<int64_t, Eigen::Isometry3d> FindSimilarPointCloud(FeatureContainer const& scan_in) {
        if (polarcontext_ringkey_tree_ == nullptr) {
            return std::make_pair(-1, Eigen::Isometry3d::Identity()); 
        }
        
        pcl::PointCloud<_PointCloudT> selected_pointcloud; 
        extractInterestPointClouds(scan_in, selected_pointcloud);
        Eigen::MatrixXd sc_desc = sc_.MakeScanContext(selected_pointcloud);                  // v1 提取sc全局特征描述符
        Eigen::MatrixXd ringkey = sc_.MakeRingkeyFromScanContext(sc_desc);   // 求 ring key 特征  
        std::vector<float> polarcontext_ringkey = eig2vec(ringkey);
        return descFindSimilar(polarcontext_ringkey, sc_desc); // <id, 相对变换>
    }

    /**
     * @brief: 闭环检测
     * @details: 查找 指定index关键帧的 闭环帧  
     * @param index
     * @return <闭环的全局index, 预测位姿关系>
     */            
    std::pair<int64_t, Eigen::Isometry3d> LoopDetect(uint32_t const& index) {
        std::cout << SlamLib::color::YELLOW << "场景识别，当前帧index: " << index << std::endl;
        Eigen::Isometry3d relpose = Eigen::Isometry3d::Identity();   
        std::lock_guard<std::mutex> lock(mt_);  
        // 首先将sc描述子提取出来  
        auto curr_ringkey = polarcontext_ringkeys_vec_[index]; // current observation (query)      提取最新一帧ring-key
        auto curr_desc = polarcontexts_sc_[index];   // current observation (query)      提取最新的sc描述子  
        /* 
        * step 1: candidates from ringkey tree_  首先查找ring key  获得候选关键帧
        */
        // 如果数量不够    不进行检测
        if(polarcontext_ringkeys_vec_.size() < NUM_EXCLUDE_RECENT_ + 1) {
            return std::pair<int64_t, Eigen::Isometry3d>(-1, relpose); // Early return 
        }

        return descFindSimilar(curr_ringkey, curr_desc); 
    } 

    /**
     * @brief: 保存sc描述子以及ringkey描述子 
     */            
    bool Save(std::string const& database_dir) {
        if(!boost::filesystem::is_directory(database_dir)) {
            boost::filesystem::create_directory(database_dir);
        }
        SlamLib::time::TicToc tt;
        uint16_t ring_num = sc_.GetRingNum();
        uint16_t sector_num = sc_.GetSectorKeyNum();
        // 循环保存所有的描述子数据
        for (uint64_t i = 0; i < polarcontexts_sc_.size(); i++) {
            // ofstream 文件输出流  用于向文件输出数据   
            std::fstream ofs(database_dir + "/KeyFrameDescriptor/scan_context_" + std::to_string(i), 
                std::ios::out | std::ios::trunc | std::ios::binary);

            lifelong_backend::scan_context::proto::scan_context sc_proto;

            for(int row = 0; row < ring_num; row++) {
                for(int col = 0; col < sector_num; col++) {
                    sc_proto.add_polarcontext(polarcontexts_sc_[i](row, col)); 
                }
            }
            
            for (uint16_t k = 0; k < polarcontext_ringkeys_vec_[i].size(); k++) {
                sc_proto.add_ringkey(polarcontext_ringkeys_vec_[i][k]);
            }

            if (!sc_proto.SerializeToOstream(&ofs)) {
                std::cerr << "sc_proto Failed to write Ostream. index: " << i << std::endl;
                continue;
            }
        }
        tt.toc("save scan-context ");
        return true; 
    }

    /**
     * @brief 
     * 
     * @param database_dir 
     * @param max_index 加载到最大的序列号
     */
    void Load(const std::string& database_dir, const uint32_t& max_index) {
        polarcontext_ringkeys_vec_.clear(); 
        polarcontexts_sc_.clear();  
        uint32_t index = 0;
        uint16_t ring_num = sc_.GetRingNum();
        uint16_t sector_num = sc_.GetSectorKeyNum();
        std::cout << "加载scan_context描述子,最大index: " << max_index << std::endl;
        // 加载描述子 
        SlamLib::time::TicToc tt;

        while(index < max_index) {
            // ifstream 文件输入流  用于向文件输入数据    
           std::fstream ifs(database_dir + "/KeyFrameDescriptor/scan_context_" + std::to_string(index), 
                                            std::ios::in | std::ios::binary);
            
            if(!ifs) {
                std::cerr << "failed to create fstream, index: " << index << std::endl;
                continue;
            }

            index++;  
            lifelong_backend::scan_context::proto::scan_context sc_proto;

            if (!sc_proto.ParseFromIstream(&ifs)) {
                std::cerr << "sc_proto failed to read fstream, index: " << index - 1 << std::endl;
                continue;
            }

            Eigen::MatrixXd sc = Eigen::MatrixXd::Zero(ring_num, sector_num);
            uint16_t n = 0; 

            for(int i = 0; i < ring_num; i++) {
                for(int j = 0; j < sector_num; j++) {
                    sc(i, j) = sc_proto.polarcontext(n);
                    ++n;  
                }
            }

            polarcontexts_sc_.push_back(std::move(sc));

            std::vector<float> ring(ring_num, 0);
            // std::cout<<"ring: "<<std::endl;
            for(int j = 0; j < ring_num; j++) {
                ring[j] = sc_proto.ringkey(j);
            }

            polarcontext_ringkeys_vec_.push_back(std::move(ring));
        }

        tt.toc("load scan-context ");
        LOG(INFO) << SlamLib::color::GREEN << "load scan-context num " 
            << index << SlamLib::color::RESET;
        
        if (polarcontext_ringkeys_vec_.empty())  
            return;  

        // 构建KDTREE
        tt.tic();  
        polarcontext_ringkeys_to_search_.clear();
        // 构造用于建立kdtree的历史ringkey集合     
        // 这里减去 NUM_EXCLUDE_RECENT_ 也就是 不考虑最近的若干帧 
        polarcontext_ringkeys_to_search_.assign( // [first，end)
            polarcontext_ringkeys_vec_.begin(), 
            polarcontext_ringkeys_vec_.end());  // end() 指向 最后一个元素的后一个
        // KDTreeVectorOfVectorsAdaptor<>的 unique_ptr 
        polarcontext_ringkey_tree_.reset(
            new ringKeyKdtree(sc_.GetRingNum() ,// ring的维度 例如20     
                                                        polarcontext_ringkeys_to_search_, 
                                                        // 最后结点的最大叶子数   即kdtree最后一个结点包含元素的最大个数
                                                        10 )); 
        tt.toc("build scan-context kdtree ");
        LOG(INFO) << SlamLib::color::GREEN << "位置识别数据库加载完成!"<< SlamLib::color::RESET;

        return; 
    }

protected:
    /**
     * @brief 在输入的点云特征中提取出感兴趣的
     * 
     * @param pcl_in 
     * @param selected_pointcloud 
     */
    void extractInterestPointClouds(const FeatureContainer& pcl_in, 
            pcl::PointCloud<_PointCloudT>& selected_pointcloud) {
        selected_pointcloud.clear(); 
        // 如果没有指定 目标点云的名称   那么 map 中全部点云  都进行使用 
        if (target_names_.empty()) {
            // 遍历点云数据   将所有点云数据合并 
            for (auto iter = pcl_in.begin(); iter != pcl_in.end(); ++iter) {
                selected_pointcloud += *(iter->second);
            }
        } else {
            for (std::string const& name : target_names_) {
                if (pcl_in.find(name) != pcl_in.end()) {
                    selected_pointcloud += *pcl_in.at(name);
                }
            }
        }
    }

    /**
     * @brief: 输入 描述子 在数据库中找到相似的一帧
     * @details 步骤： 1、先通过 ringkey 找到 候选帧   2、
     * @param ring_key ring key 描述子
     * @param sc_desc SC描述子 
     * @return <index, 相对变换>
     */            
    std::pair<int64_t, Eigen::Isometry3d> descFindSimilar( const std::vector<float>& ring_key, 
                                                                                                                         const Eigen::MatrixXd& sc_desc) {
        std::cout << "descFindSimilar" << std::endl;
        double min_dist = 10000000; // init with somthing large
        int nn_align = 0;
        int64_t nn_idx = 0;
        // knn search   NUM_CANDIDATES_FROM_TREE_ = 10  , 10个候选关键帧    
        std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE_);   // 保存候选帧在 数据容器中的序号 
        std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE_);        // 保存候选关键帧的ringkey的距离  

        // SlamLib::time::TicToc t_tree_search;
        // 找 NUM_CANDIDATES_FROM_TREE_  个最近关键帧 
        nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE_);
        // 初始化    用 candidate_indexes 和  out_dists_sqr 数组的数组名地址初始化          设置搜索结果存放的数组  
        knnsearch_result.init( &candidate_indexes[0],    // 存放序号的数组
                                                        &out_dists_sqr[0] );    // 存放 距离的数组     
        // 查找与当前待搜索的ringkey 距离最近的10帧
        polarcontext_ringkey_tree_->index->findNeighbors(knnsearch_result, 
                                                                                                    &ring_key[0],  // 当前的描述子 
                                                                                                    nanoflann::SearchParams(10)); 
        // t_tree_search.toc("Tree search");
        /* 
        *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)    
                    挑选最佳的候选关键帧   旋转检查
        */
        // SlamLib::time::TicToc t_calc_dist;   
        // 遍历所有候选关键帧       找到距离最近的index 与 旋转量   
        for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE_; candidate_iter_idx++ ) {   
            // 获取候选关键帧 的sc描述子  
            Eigen::MatrixXd polarcontext_candidate = polarcontexts_sc_[candidate_indexes[candidate_iter_idx]];
            // 计算描述子的距离  
            std::pair<double, int> sc_dist_result = sc_.DistanceBtnScanContext(sc_desc, polarcontext_candidate); 
            
            double candidate_dist = sc_dist_result.first;       // 距离   
            int candidate_align = sc_dist_result.second;        // 平移量 
            // 如果距离最小  
            if( candidate_dist < min_dist ) {
                min_dist = candidate_dist;
                nn_align = candidate_align;
                // 获取index  
                nn_idx = candidate_indexes[candidate_iter_idx];
            }
        }
        // t_calc_dist.toc("Distance calc");
        int64_t loop_idx = -1; 
        Eigen::Isometry3d relpose = Eigen::Isometry3d::Identity();   
        /* 
        * loop threshold check    即判断是否小于阈值  
        */
        if (min_dist < SC_DIST_THRES_) {
            loop_idx = nn_idx; 
            // std::std::cout.precision(3); 
            std::cout << SlamLib::color::GREEN << "[Loop found] Nearest distance: " 
                << min_dist << " match idx: " << nn_idx << std::endl;
            std::cout << "[Loop found] yaw diff: " << nn_align * sc_.PC_UNIT_SECTORANGLE_ 
                << " deg." << SlamLib::color::RESET << std::endl;
        } else {
            std::cout.precision(3); 
            std::cout << "[Not loop] Nearest distance: " << min_dist << " match idx: " << nn_idx<< "." << std::endl;
            std::cout << "[Not loop] yaw diff: " << nn_align * sc_.PC_UNIT_SECTORANGLE_ << " deg." 
                << SlamLib::color::RESET << std::endl;
            return std::pair<int64_t, Eigen::Isometry3d>(-1, relpose); 
        }
        // 转换为变换矩阵
        float yaw_diff_rad = deg2rad(nn_align * sc_.PC_UNIT_SECTORANGLE_);      // yaw角的差
        relpose.translation() = Eigen::Vector3d(0, 0, 0);
        Eigen::Matrix3d rot;
        rot = Eigen::AngleAxisd(-yaw_diff_rad, Eigen::Vector3d::UnitZ()) 
                    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) 
                    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        relpose.linear() = rot;  
        return std::make_pair(loop_idx, relpose);   
    }

    /**
     * @brief 
     * 
     * @param radians 
     * @return float 
     */
    float rad2deg(float radians) {
        return radians * 180.0 / M_PI;
    }
    
    /**
     * @brief 
     * 
     * @param degrees 
     * @return float 
     */
    float deg2rad(float degrees) {
        return degrees * M_PI / 180.0;
    }
    
    /**
     * @brief eigen 类型数据转换为 vector 数据 
     * 
     * @param _eigmat 
     * @return std::vector<float> 
     */
    std::vector<float> eig2vec(Eigen::MatrixXd _eigmat) {
        std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size());
        return vec;
    } 
private:
    const int TREE_MAKING_PERIOD_ = 10;  // 更新ringkey kdtree的帧间隔   
    const int NUM_EXCLUDE_RECENT_ = 50;  // 当前帧前 NUM_EXCLUDE_RECENT_之内 不参与回环检测    
    // 构建搜索树的间隔  
    const int NUM_CANDIDATES_FROM_TREE_ = 10; // 10 is enough. (refer the IROS 18 paper)
    // 相似距离阈值  
    const double SC_DIST_THRES_ = 0.2; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
    // 每一帧的SC描述子
    std::deque<Eigen::MatrixXd> polarcontexts_sc_;
    // vector 类型的 ring-key 
    KeyVec polarcontext_ringkeys_vec_;
    KeyVec polarcontext_ringkeys_to_search_;
    // kdtree 指针 
    std::unique_ptr<ringKeyKdtree> polarcontext_ringkey_tree_; 
    // config 
    int  tree_making_period_conter_ = 0;

    std::vector<std::string> target_names_;   // 选择的点云表示名
    ScanContext<_PointCloudT> sc_;   
    std::mutex mt_;  
}; // class 
} // namespace 
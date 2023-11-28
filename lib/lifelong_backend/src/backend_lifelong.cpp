/**
 * @file backend_lifelong.cpp
 * @brief 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2023 
 * 
 */
#define NDEBUG
#include "lifelong_backend/backend_lifelong_impl.hpp"

namespace lifelong_backend {
//模板类需要特化
template class LifeLongBackEndOptimization<pcl::PointXYZI>;
} // namespace 

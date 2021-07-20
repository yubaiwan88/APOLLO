/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file dp_st_graph.cc
 **/

#include "modules/planning/tasks/dp_st_speed/dp_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_thread_pool.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::VehicleParam;
using apollo::common::math::Vec2d;

namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();

// 判断起始点到当前点的连线是否与stboundary有重叠
bool CheckOverlapOnDpStGraph(const std::vector<const StBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  const common::math::LineSegment2d seg(p1.point(), p2.point());
  for (const auto* boundary : boundaries) {
    // 这个if语句在判断boundary的类型，如果是KEEP_CLEAR类型，说明这个boundary不会影响车辆的行进，不予考虑，进行下一次循环。
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    if (boundary->HasOverlap(seg)) {
      return true;
    }
  }
  return false;
}
}  // namespace

DpStGraph::DpStGraph(const StGraphData& st_graph_data,
                     const DpStSpeedConfig& dp_config,
                     const std::vector<const PathObstacle*>& obstacles,
                     const common::TrajectoryPoint& init_point,
                     const SLBoundary& adc_sl_boundary)
    : st_graph_data_(st_graph_data),
      dp_st_speed_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, obstacles, init_point_),
      adc_sl_boundary_(adc_sl_boundary) {
  dp_st_speed_config_.set_total_path_length(std::fmin(dp_st_speed_config_.total_path_length(),st_graph_data_.path_data_length()));
  unit_s_ = dp_st_speed_config_.total_path_length() /(dp_st_speed_config_.matrix_dimension_s() - 1); //s方向的分辨率
  unit_t_ = dp_st_speed_config_.total_time() /(dp_st_speed_config_.matrix_dimension_t() - 1); //t方向的分辨率
}

Status DpStGraph::Search(SpeedData* const speed_data) {
  constexpr float kBounadryEpsilon = 1e-2;
  // 第一部分是一个for循环，遍历的是STGraph中所有的boundary，
  // 每一个障碍物都会生成一个对应的boundary，这里就是将所有障碍的boundary都判断一遍。
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    // keep_clear的范围不影响速度规划,在 Dp St 决策中不考虑 KeepClear 
    //  这个if语句在判断boundary的类型，如果是KEEP_CLEAR类型，说明这个boundary不会影响车辆的行进，不予考虑，进行下一次循环。
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    //若boundary包含原点或非常接近原点（即自车现在的位置），自车应该stop，不再前进
    //故s,v,a都是0，即随着t推移，自车不动
    /*
     对于类型不是KEEP_CLEAR的boundary，进行boundary与车辆起点位置（init_point）的判断。
     ST图的起点，即车辆的初始位置，设定为(0,0)。如果起点在boundary的范围内，或者boundary十分接近，认为车辆此时不能移动，
     因此，在整个时间范围[0,t]内，设置车辆速度v为0，加速度a为0，位置s为0。
    */ 
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||(std::fabs(boundary->min_t()) < kBounadryEpsilon &&
                                                                                                    std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      std::vector<SpeedPoint> speed_profile;
      float t = 0.0;
      for (int i = 0; i < dp_st_speed_config_.matrix_dimension_t();++i, t += unit_t_) {
        SpeedPoint speed_point;
        speed_point.set_s(0.0);
        speed_point.set_t(t);
        speed_profile.emplace_back(speed_point);
      }
      speed_data->set_speed_vector(speed_profile);
      return Status::OK();
    }
  }

// 没有障碍物，dp_st_graph输出默认的速度曲线
  if (st_graph_data_.st_boundaries().empty()) {
    ADEBUG << "No path obstacles, dp_st_graph output default speed profile.";
    std::vector<SpeedPoint> speed_profile;
    float s = 0.0;
    float t = 0.0;
    for (int i = 0; i < dp_st_speed_config_.matrix_dimension_t() &&i < dp_st_speed_config_.matrix_dimension_s();++i, t += unit_t_, s += unit_s_)
    {
      SpeedPoint speed_point;
      speed_point.set_s(s);
      speed_point.set_t(t);
      const float v_default = unit_s_ / unit_t_;
      speed_point.set_v(v_default);
      speed_point.set_a(0.0);
      // std::move(speed_point)将speed_point中值移动，使speed_point变为空
      speed_profile.emplace_back(std::move(speed_point));
    }
    speed_data->set_speed_vector(std::move(speed_profile));
    return Status::OK();
  }

// 初始化cost_table
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
// 计算所有的cost，更新cost_table_
  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
// 分配速度
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

// 构建一个costTable,并初始化为具体行和列的st中的坐标
Status DpStGraph::InitCostTable() {
  uint32_t dim_s = dp_st_speed_config_.matrix_dimension_s();
  uint32_t dim_t = dp_st_speed_config_.matrix_dimension_t();
  DCHECK_GT(dim_s, 2);
  DCHECK_GT(dim_t, 2);
  // 每个元素都是StGraphPoint类型的StGraphPoint()。
    // cost_table_[t][s]-------  // row: s, col: t --- NOTICE: Please do NOT change.，表中t为列放在外层，s为行放在内层
  cost_table_ = std::vector<std::vector<StGraphPoint>>(dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

  float curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    float curr_s = 0.0;
    for (uint32_t j = 0; j < cost_table_i.size(); ++j, curr_s += unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }
  return Status::OK();
}

/*
这个函数是进行动态规划的主体，通过两层for循环，遍历CostTable中的点，计算每个点的cost。
每个点的总的cost称作total_cost，一个点的total_cost由四部分相加构成：
      1.关于障碍物的cost——obstacle_cost；
      2.关于空间位置的cost——spatial_potential_cost；
      3.前一个点的total_cost；
      4.EdgeCost，EdgeCost由三部分构成：（1）Speedcost，（2）AccelCost，（3）JerkCost。
*/ 
// 计算每个点的cost
Status DpStGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  /*
  首先定义了两个变量，这两个变量限定的是下一列的s范围。因为我们在初始化CostTable时，
  每个时间点上的采样范围都是[0,total_s_]，在实际的计算过程中，并不需要每次都遍历所有的点，
  因此，在通过函数CalculateCostAt()计算当前点的cost过后，
  需要通过函数GetRowRange()计算出下一列的s的范围[next_lowest_row,next_highest_row]。
  */ 
  uint32_t next_highest_row = 0;
  uint32_t next_lowest_row = 0;

  for (size_t c = 0; c < cost_table_.size(); ++c) {
    int highest_row = 0;
    int lowest_row = cost_table_.back().size() - 1;

    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {
      // 在这两层循环中，调用CalculateCostAt()，在每个时刻t，遍历该时刻所有的采样点，
      // 采样点的s范围为[next_lowest_row,next_highest_row]。
      if (FLAGS_enable_multi_thread_in_dp_st_graph) {
        PlanningThreadPool::instance()->Push(td::bind(&DpStGraph::CalculateCostAt, this, c, r));
      } else {
        CalculateCostAt(c, r);
      }
    }
    if (FLAGS_enable_multi_thread_in_dp_st_graph) {
      PlanningThreadPool::instance()->Synchronize();
    }

    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<float>::infinity()) {
        int h_r = 0;
        int l_r = 0;
        // 计算下一列的s的范围
        GetRowRange(cost_cr, &h_r, &l_r);
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return Status::OK();
}

void DpStGraph::GetRowRange(const StGraphPoint& point, int* next_highest_row,
                            int* next_lowest_row) {
  float v0 = 0.0;
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = (point.index_s() - point.pre_point()->index_s()) * unit_s_ / unit_t_;
  }

  const int max_s_size = cost_table_.back().size() - 1;

  const float speed_coeff = unit_t_ * unit_t_;

  const float delta_s_upper_bound = v0 * unit_t_ + vehicle_param_.max_acceleration() * speed_coeff;
  *next_highest_row = point.index_s() + static_cast<int>(delta_s_upper_bound / unit_s_);
  if (*next_highest_row >= max_s_size) {
    *next_highest_row = max_s_size;
  }

  const float delta_s_lower_bound = std::fmax(
      0.0, v0 * unit_t_ + vehicle_param_.max_deceleration() * speed_coeff);
  *next_lowest_row =
      point.index_s() + static_cast<int>(delta_s_lower_bound / unit_s_);
  if (*next_lowest_row > max_s_size) {
    *next_lowest_row = max_s_size;
  } else if (*next_lowest_row < 0) {
    *next_lowest_row = 0;
  }
}
/*
通过动态规划的方法进行速度规划就是在CalculateCostAt()中进行的。
CalculateCostAt()输入的参数是(c,r)。注意这里的c和r指的是列和行的序号，而不是具体坐标。
在Apollo的代码中，表示点的行列号用(c,r)，具体坐标用(curr_t,curr_s)。
CalculateCostAt()中的输入参数是[c,r]，表示的是采样点的index。
通过遍历cost_table_中所有采样点，在CalculateCostAt()中计算当前点cost_cr的TotalCost。
*/ 
void DpStGraph::CalculateCostAt(const uint32_t c, const uint32_t r) {
  auto& cost_cr = cost_table_[c][r]; //当前采样点
  // 首先，通过GetObstacleCost()计算采样点的obstacle_cost，如果obstacle_cost无穷大，则返回，计算下一个点。
// 通过GetSpatialPotentialCost()计算SpatialPotentialCost，这部分反映的是当前点到终点的距离的cost
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
  if (cost_cr.obstacle_cost() > std::numeric_limits<float>::max()) {
    return;
  }
//  定义cost_table_的第一个点为起始点。
  const auto& cost_init = cost_table_[0][0];
  if (c == 0) {
    DCHECK_EQ(r, 0) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    return;
  }
// unit_s_ * t代表路径长度
  float speed_limit = st_graph_data_.speed_limit().GetSpeedLimitByS(unit_s_ * r);
  /*
     当c==1，表示计算第二列的点。
     首先，判断当前点的加速度和减速度是否超出范围。
     接下来，判断起始点到当前点的连线是否与stboundary有重叠。
     如果通过上述两项考察，计算当前点的TotalCost。
     因为第二列的上一列中只有起始点，所以，将第二列中的采样点的PrePoint设置为起始点cost_init。
  */ 
  if (c == 1) {
    const float acc = (r * unit_s_ / unit_t_ - init_point_.v()) / unit_t_;
    if (acc < dp_st_speed_config_.max_deceleration() ||
        acc > dp_st_speed_config_.max_acceleration()) {
      return;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,cost_init)) {
      return;
    }
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + cost_init.total_cost() +CalculateEdgeCostForSecondCol(r, speed_limit));
    cost_cr.SetPrePoint(cost_init); 
    return;
  }

// 进一步缩小上一列的计算范围
  constexpr float kSpeedRangeBuffer = 0.20;
  const uint32_t max_s_diff = static_cast<uint32_t>(FLAGS_planning_upper_speed_limit *(1 + kSpeedRangeBuffer) * unit_t_ / unit_s_);
  const uint32_t r_low = (max_s_diff < r ? r - max_s_diff : 0);

  const auto& pre_col = cost_table_[c - 1];
/*
     在计算第三列(c==2)之前，需要进一步缩小第二列的计算范围，上述代码完成的就是这一项工作。
     缩小前一列的计算范围，主要是确定上界和下界。上界很好理解，当前点的r就是上一列的上界。
     确定下界，需要从当前点回溯上一列，当前的的s坐标-最大车速×unit_t_，就能够得到上一列s的最小值pre_lowest_s。
     根据前面得到的存储采样点index的vector spatial_distance_by_index_，
     就能够得到上一列的计算范围[r_low, r]，上一列的size，r_pre_size = r - r_low + 1，
*/ 
/*
在完成缩小前一列计算范围的工作之后，计算第三列采样点的TotalCost。
遍历前一列[r_low, r]范围内的点，计算，由起始点开始，经第二列的点，到达当前点的TotalCost，计算方法同上面的方法相同。
不同之处在于，由于当前点可经由第二列不同的点到达，因此，会得到几个TotalCost，从这些TotalCost中找到最小的一个，
并记录下对应的PrePoint，记为当前点的PrePoint，
*/ 
  if (c == 2) {
    for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
      const float acc =(r * unit_s_ - 2 * r_pre * unit_s_) / (unit_t_ * unit_t_);
      if (acc < dp_st_speed_config_.max_deceleration() ||
          acc > dp_st_speed_config_.max_acceleration()) {
        continue;
      }

      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,pre_col[r_pre])) {
        continue;
      }

      const float cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() + CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);
      // 记录从初始点经过第二列采样点到第三列采样点最小的cost,记录前一个点（第二个点，代价最小）
      // 保存当前点最小代价，和记录前一个点（使代价最小的前点）
      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
      }
    }
    return;
  }
/*
 当计算进行到第4列及以后，就会执行这部分代码。
 同样的，在计算之前，已经进行了缩小前一列范围的工作，这里遍历前一列[r_low, r]范围内的点，计算当前点的cost。
 这一部分的计算方法与c=2时的是相同的，之所以再写一遍是计算JerkCost时，这里会多计算一个点，仅此处不同而已。
 同样的，在最后会找到当前列TotalCost最小的点，并记录下它的PrePoint。
*/ 
  for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
    if (std::isinf(pre_col[r_pre].total_cost()) ||pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }

    const float curr_a = (cost_cr.index_s() * unit_s_ +pre_col[r_pre].pre_point()->index_s() * unit_s_ -
                          2 * pre_col[r_pre].index_s() * unit_s_) /
                         (unit_t_ * unit_t_);
    if (curr_a > vehicle_param_.max_acceleration() ||
        curr_a < vehicle_param_.max_deceleration()) {
      continue;
    }
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }

    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();
    float cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                                                                                                                                                                    curr_point, speed_limit);

    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
    }
  }
}

// 在动态规划之后，来得到速度曲线
Status DpStGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  float min_cost = std::numeric_limits<float>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  // 经过CalculateCostAt()的计算，我们得到了最后一列所有点的TotalCost，
  // 通过这个for循环，我们可以找到最后一列TotalCost最小的采样点，将这个点暂定为最佳的终点，记为best_end_point，
  // 并记下它的TotalCost为min_cost。
  /*
  这里需要插一句，为什么不直接将最后一列TotalCost最小的点定位最佳终点呢？因为采样时间是我们估计的一个大概的时间，
  在这个时间之前，动态规划可能已经规划到了路径的终点（s终点），只是因为还没有计算到cost_table_的最后一列，才一直进行计算。
  因此，我们在判断为最后一列之后，需要继续判断每一列的最后一个点，是不是已经到达了路径的终点。
  */ 
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) &&cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost(); //找到最后一列cost最小的点
    }
  }
// 遍历每一列的最后一个点，将其TotalCost与上一步的min_cost做比较，TotalCost更小的点确定为真正的best_end_point。
  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

// 在确定了终点之后，我们就可以顺藤摸瓜（自下而上），找到从起点到终点的所有采样点了。
  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());//找到从起始点到达最佳终点的所有点组成的序列
// 有了所有采样点的(t, s)坐标，根据牛顿第二定律，可以确定每个点的速度，这样就得到了速度规划的速度文件。
  constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
  if (speed_profile.front().t() > kEpsilon || speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  speed_data->set_speed_vector(speed_profile);
  return Status::OK();
}

float DpStGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second,
                                   const STPoint& third, const STPoint& forth,
                                   const float speed_limit) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

float DpStGraph::CalculateEdgeCostForSecondCol(const uint32_t row,
                                               const float speed_limit) {
  float init_speed = init_point_.v();
  float init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

float DpStGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                              const uint32_t pre_row,
                                              const float speed_limit) {
  float init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo

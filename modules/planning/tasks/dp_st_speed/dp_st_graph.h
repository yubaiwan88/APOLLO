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
 * @file dp_st_graph.h
 **/

#ifndef MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_
#define MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_

#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/path_obstacle.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/tasks/dp_st_speed/dp_st_cost.h"
#include "modules/planning/tasks/dp_st_speed/st_graph_point.h"
#include "modules/planning/tasks/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

class DpStGraph {
 public:
  DpStGraph(const StGraphData& st_graph_data, const DpStSpeedConfig& dp_config,
            const std::vector<const PathObstacle*>& obstacles,
            const common::TrajectoryPoint& init_point,
            const SLBoundary& adc_sl_boundary);

  apollo::common::Status Search(SpeedData* const speed_data);
// 在对自车当前位置（原点）判断后，速度规划主要分为3步：
// 1、构建代价表InitCostTable()，
// 2、更新代价表中每个节点的代价CalculateTotalCost()，
// 3、查找代价最小的规划结束点并逆向构建路径、求取速度RetrieveSpeedProfile()。
 private:
//  主要小心cost_table_外层是t，内层是s，行数是s，列数是t
  apollo::common::Status InitCostTable();

  apollo::common::Status RetrieveSpeedProfile(SpeedData* const speed_data);


  // DpStGraph::CalculateTotalCost()中是对每一行、每一列的遍历，
  // 巧的地方在于使用了GetRowRange()来计算由当前节点可到达的s（行）范围。

// DpStGraph::CalculateTotalCost()的目的是对cost_table_中的每个节点更新其cost，
// 在此，我们先说如何更新单个节点的cost，请看DpStGraph::CalculateCostAt()函数，动态规划的核心都在这个函数里。
  apollo::common::Status CalculateTotalCost();
  void CalculateCostAt(const uint32_t r, const uint32_t c);

  float CalculateEdgeCost(const STPoint& first, const STPoint& second,
                           const STPoint& third, const STPoint& forth,
                           const float speed_limit);
  float CalculateEdgeCostForSecondCol(const uint32_t row,
                                       const float speed_limit);
  float CalculateEdgeCostForThirdCol(const uint32_t curr_r,
                                      const uint32_t pre_r,
                                      const float speed_limit);
/*
CalculateCostAt()和GetRowRange()中都有基于当前点，判断可到达s（行）范围的操作。
不同的是，GetRowRange()中是向s增大的方向查找，因为要逐行的扩展节点、更新其cost。
而CalculateCostAt()中是向s减小的方向查找，即查找哪些行可以到达该点，
因为该函数用来计算某节点的cost，就要基于之前经过的节点的cost。

只以DpStCost::GetAccelCost()函数为例，提一下动态规划中常用的计算结果缓存技巧。
具体请参考apollo\modules\planning\tasks\optimizers\dp_st_speed\dp_st_cost.h
*/ 
  void GetRowRange(const StGraphPoint& point, int* highest_row,int* lowest_row);

 private:
  const StGraphData& st_graph_data_;

  // dp st configuration
  DpStSpeedConfig dp_st_speed_config_;

  // obstacles based on the current reference line 基于当前参考线的障碍物
  const std::vector<const PathObstacle*>& obstacles_;

  // vehicle configuration parameter  车辆配置参数
  const common::VehicleParam& vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();

  // initial status  最初的状态
  common::TrajectoryPoint init_point_;

  // cost utility with configuration;配置cost实用程序
  DpStCost dp_st_cost_;

  const SLBoundary& adc_sl_boundary_;

  float unit_s_ = 0.0;
  float unit_t_ = 0.0;

  // cost_table_[t][s]-------
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<StGraphPoint>> cost_table_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_DP_ST_GRAPH_H_
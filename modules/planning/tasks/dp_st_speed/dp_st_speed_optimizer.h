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
 * @file dp_st_speed_optimizer.h
 **/

#ifndef MODULES_PLANNING_TASKS_DP_ST_SPEED_OPTIMIZER_H_
#define MODULES_PLANNING_TASKS_DP_ST_SPEED_OPTIMIZER_H_

#include <string>

#include "modules/planning/proto/dp_st_speed_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/st_boundary_config.pb.h"

#include "modules/planning/tasks/speed_optimizer.h"
#include "modules/planning/tasks/st_graph/speed_limit_decider.h"
#include "modules/planning/tasks/st_graph/st_boundary_mapper.h"

namespace apollo {
namespace planning {
/*
DpStSpeedOptimizer类继承自SpeedOptimizer，后者又继承自Task，其作用是对当前速度进行动态规划ST坐标优化处理。
Task类提供两个虚函数Init、Execute，其中Init函数用于任务对象的初始化，Task类目前未做任何实际工作；
Execute用于执行实际的优化，Task类也未实现任何功能。SpeedOptimizer类重载了Task类的虚函数Execute，
同时提供一个保护的纯虚函数Process（由派生类具体实现），在Execute内部调用Process函数完成实际的优化，
如果优化失败，则调用GenerateStopProfile函数生成停车速度信息。
注意：SpeedOptimizer是虚基类，不可直接使用它创建类对象，必须从该类派生出具体类后方可实例化。

Task->init()任务对象的初始化，Execute()->SpeedOptimizer::Execute()中执行Process()->由DpSteedOptimizer类Process执行实际的优化
*/ 
/**
 * @class DpStSpeedOptimizer
 * @brief DpStSpeedOptimizer does ST graph speed planning with dynamic
 * programming algorithm.
 */
class DpStSpeedOptimizer : public SpeedOptimizer {
  DpStSpeedOptimizer();
// 从配置文件中读取相关信息，获取算法所需的各项参数初始化
  bool Init(const PlanningConfig& config) override;

 private:
//  首先获取路径边界boundary，然后创建一个st_graph,然后调用search函数执行图搜索获取最优速度信息
  apollo::common::Status Process(const SLBoundary& adc_sl_boundary,
                                 const PathData& path_data,
                                 const common::TrajectoryPoint& init_point,
                                 const ReferenceLine& reference_line,
                                 const SpeedData& reference_speed_data,
                                 PathDecision* const path_decision,
                                 SpeedData* const speed_data) override;

  bool SearchStGraph(const StBoundaryMapper& boundary_mapper,
                     const SpeedLimitDecider& speed_limit_decider,
                     const PathData& path_data, SpeedData* speed_data,
                     PathDecision* path_decision,
                     planning_internal::STGraphDebug* debug) const;

 private:
  common::TrajectoryPoint init_point_;
  const ReferenceLine* reference_line_ = nullptr;
  SLBoundary adc_sl_boundary_;
  // ST坐标速度配置类对象
  DpStSpeedConfig dp_st_speed_config_;
  // ST坐标边界配置类对象
  StBoundaryConfig st_boundary_config_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_DP_ST_SPEED_OPTIMIZER_H_

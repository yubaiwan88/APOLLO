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
 * @file
 **/

#include "modules/planning/tasks/dp_poly_path/dp_poly_path_optimizer.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common/util/file.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/dp_poly_path/dp_road_graph.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

DpPolyPathOptimizer::DpPolyPathOptimizer(): PathOptimizer("DpPolyPathOptimizer") {}

// 初始化函数，将em中dp_poly_path_config中参数加载--------优化器参数加载
bool DpPolyPathOptimizer::Init(const PlanningConfig &config) {
  config_ = config.em_planner_config().dp_poly_path_config();
  is_init_ = true;
  return true;
}
/*
处理结果：最短路径就是依据若干level之间分段5次多项式的采样点，
保存在path_data.frenet_path_（SL系）和path_data.discretized_path_（XY系）中
*/ 
Status DpPolyPathOptimizer::Process(const SpeedData &speed_data,
                                    const ReferenceLine &,
                                    const common::TrajectoryPoint &init_point,
                                    PathData *const path_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process().";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }
  CHECK_NOTNULL(path_data);

  DPRoadGraph dp_road_graph(config_, *reference_line_info_, speed_data);

  dp_road_graph.SetDebugLogger(reference_line_info_->mutable_debug());
// FindPathTunnel()主要分为3部分：先设置相关前提条件，然后查找代价最小路径，
// 最后对每段代价最小路径采样以构造FrenetFramePath类的实例，并存入path_data中。
// 将最终的最优路径保存在path_data
  if (!dp_road_graph.FindPathTunnel(init_point,reference_line_info_->path_decision()->path_obstacles().Items(),path_data)) {
    AERROR << "Failed to find tunnel in road graph";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph path generation");
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo

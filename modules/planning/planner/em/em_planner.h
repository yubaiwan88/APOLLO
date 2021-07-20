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

#ifndef MODULES_PLANNING_PLANNER_EM_EM_PLANNER_H_
#define MODULES_PLANNING_PLANNER_EM_EM_PLANNER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/planner/planner.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/planning/tasks/task.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class EMPlanner
 * @brief EMPlanner is an expectation maximization planner.
 */

class EMPlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  EMPlanner() = default;

  /**
   * @brief Destructor
   */
  virtual ~EMPlanner() = default;

  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  apollo::common::Status Plan(const common::TrajectoryPoint& planning_init_point,Frame* frame) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;

 private:
  void RegisterTasks();

// 生成初始速度
  std::vector<common::SpeedPoint> GenerateInitSpeedProfile(
      const common::TrajectoryPoint& planning_init_point,
      const ReferenceLineInfo* reference_line_info);

  std::vector<common::SpeedPoint> DummyHotStart(const common::TrajectoryPoint& planning_init_point);

  std::vector<common::SpeedPoint> GenerateSpeedHotStart(const common::TrajectoryPoint& planning_init_point);

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,PathData* path_data);

  void GenerateFallbackSpeedProfile(const ReferenceLineInfo* reference_line_info, SpeedData* speed_data);

  SpeedData GenerateStopProfile(const double init_speed,const double init_acc) const;

  SpeedData GenerateStopProfileFromPolynomial(const double init_speed,const double init_acc) const;

  bool IsValidProfile(const QuinticPolynomialCurve1d& curve) const;

  common::SLPoint GetStopSL(const ObjectStop& stop_decision,const ReferenceLine& reference_line) const;

  void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,const std::string& name, const double time_diff_ms);
/*
1、任务工厂类对象（apollo::common::util::Factory<TaskType, Task> task_factory_）
该对象的负责是注册所有的任务类，并根据配置信息生成合适的任务类对象。
2、任务类对象数组（std::vector<std::unique_ptr<Task>> tasks_）
该数组的职能是保存各个任务类对象的智能指针。
*/ 
  apollo::common::util::Factory<TaskType, Task> task_factory_; //注册所有任务类，根据配置信息生成合适的任务类对象
  std::vector<std::unique_ptr<Task>> tasks_;  //保存各个任务类对象
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_EM_EM_PLANNER_H_

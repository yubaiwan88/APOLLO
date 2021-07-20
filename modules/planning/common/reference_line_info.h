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
 * referencelineInfo类用于存储参考线信息。
 * 所谓参考线实际就是一条驾驶路径，因此参考线信息实际就是一条驾驶路径所需的各种信息
 * 包括：车辆状态、当前轨迹点、路径数据、速度数据等
 * 
 * ReferenceLineInfo不仅仅包含了参考线信息，还包含了车辆状态，路径信息，速度信息，决策信息以及轨迹信息等。
 * @file
 **/

#ifndef MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_
#define MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_

#include <algorithm>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
  explicit ReferenceLineInfo(const common::VehicleState& vehicle_state,
                             const common::TrajectoryPoint& adc_planning_point,
                             const ReferenceLine& reference_line,
                             const hdmap::RouteSegments& segments);
/*
检查无人车是否在参考线上
需要满足无人车对应的边界框start_s和end_s在参考线[0, total_length]区间内

检查无人车是否离参考线过远
需要满足无人车start_l和end_l在[-kOutOfReferenceLineL, kOutOfReferenceLineL]区间内，其中kOutOfReferenceLineL取10

将障碍物信息加入到ReferenceLineInfo类中
除了将障碍物信息加入到类中，还有一个重要的工作就是确定某个时间点无人车能前进到的位置，如下图所示。

可以看到这个过程其实就是根据障碍物的轨迹(某个相对时间点，障碍物运动到哪个坐标位置)，
并结合无人车查询得到的理想路径，得到某个时间点low_t和high_t无人车行驶距离的下界low_s-adc_start_s和上界high_s-adc_start_s
*/ 
  bool Init(const std::vector<const Obstacle*>& obstacles);

  bool IsInited() const;
// 在当前参考线中加入障碍，让参考线更为接近实际的驾驶路径
  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  PathObstacle* AddObstacle(const Obstacle* obstacle);
  void AddObstacleHelper(const Obstacle* obstacle, int* ret);

  PathDecision* path_decision();
  const PathDecision& path_decision() const;
  const ReferenceLine& reference_line() const;
  const common::TrajectoryPoint& AdcPlanningPoint() const;
// 用于判断当前参考线是否抵达终点
  bool ReachedDestination() const;

  void SetTrajectory(const DiscretizedTrajectory& trajectory);

  const DiscretizedTrajectory& trajectory() const;
  double TrajectoryLength() const;

  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  // For lattice planner'speed planning target
  void SetStopPoint(const StopPoint& stop_point);
  void SetCruiseSpeed(double speed);
  const PlanningTarget& planning_target() const { return planning_target_; }

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  // 用于判断当前参考线是否起始于上一条参考线
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  planning_internal::Debug* mutable_debug() { return &debug_; }
  const planning_internal::Debug& debug() const { return debug_; }
  LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  const LatencyStats& latency_stats() const { return latency_stats_; }

  const PathData& path_data() const;
  const SpeedData& speed_data() const;
  PathData* mutable_path_data();
  SpeedData* mutable_speed_data();
  // aggregate final result together by some configuration
  // 将当前参考线的路径数据和速度信息综合考虑，形成实际可供驾驶的路径
  bool CombinePathAndSpeedProfile(
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);
// 
  const SLBoundary& AdcSlBoundary() const;
  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  // 用于判断当前参考线是否是一条改变车道的参考线，也就是说，车道当前位置不在参考线上
  bool IsChangeLanePath() const;

  /**
   * Check if the current reference line is the neighbor of the vehicle 
   * current position
   * 检查当前参考线是否与车辆相邻
   * 当前位置
   */
  bool IsNeighborLanePath() const;

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   * 设置车辆是否可以沿此基准线行驶
  * 如果参考线正常，planner需要将此值设置为true
   */
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  void ExportEngageAdvice(common::EngageAdvice* engage_advice) const;

  bool IsSafeToChangeLane() const { return is_safe_to_change_lane_; }

  const hdmap::RouteSegments& Lanes() const;
  const std::list<hdmap::Id> TargetLaneId() const;
// 输出路径决策给其他函数调用
  void ExportDecision(DecisionResult* decision_result) const;

  void SetJunctionRightOfWay(double junction_s, bool is_protected);

  ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

  bool IsRightTurnPath() const;

  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

 private:
  bool CheckChangeLane() const;

  void ExportTurnSignal(common::VehicleSignal* signal) const;

  bool IsUnrelaventObstacle(PathObstacle* path_obstacle);

  void MakeDecision(DecisionResult* decision_result) const;
  int MakeMainStopDecision(DecisionResult* decision_result) const;
  void MakeMainMissionCompleteDecision(DecisionResult* decision_result) const;
  void MakeEStopDecision(DecisionResult* decision_result) const;
  void SetObjectDecisions(ObjectDecisions* object_decisions) const;
  // 车辆状态类对象  -------- 从其他地方获取的车辆状态
  const common::VehicleState vehicle_state_;
  // 规划轨迹点类对象 ---------- 从其他地方获取的规划点
  const common::TrajectoryPoint adc_planning_point_;
  //  参考线类对象 ------------ 从其他地方获取的参考线
  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  // 参考线的代价 ----- 参考线代价，越小越好
  double cost_ = 0.0;

  bool is_inited_ = false;
// 能否驾驶 ------- 表示参考线能否驾驶
  bool is_drivable_ = true;
// 路径决策类对象 ------------ 判断当前参考线是否有障碍
  PathDecision path_decision_;
// 路径数据类对象 ------------- 存储参考线包含的路径信息
  PathData path_data_;
  // 速度数据类对象 ------------ 存储参考线包含的速度信息
  SpeedData speed_data_;
// 离散化轨迹类对象 ----------- 存储参考线的离散化轨迹点集
  DiscretizedTrajectory discretized_trajectory_;
// SL边界类对象
  SLBoundary adc_sl_boundary_;

  planning_internal::Debug debug_;
  // 隐藏状态类对象  ----------- 存储参考线包含的隐藏状态，包含总时长，控制器时长
  LatencyStats latency_stats_;
// 车道路径片段类对象  ------------ 存储参考线的车道片段集
  hdmap::RouteSegments lanes_;

  bool is_on_reference_line_ = false;

  bool is_safe_to_change_lane_ = false;

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;

  DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_REFERENCE_LINE_INFO_H_

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

#ifndef MODULES_PLANNING_COMMON_FRAME_H_
#define MODULES_PLANNING_COMMON_FRAME_H_

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/change_lane_decider.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/lag_prediction.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/reference_line/reference_line_provider.h"

namespace apollo {
namespace planning {

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 * Frame类用于存储规划模块用到的各类信息，包括起始点，车辆状态，参考线信息列表（即多条候选路径）
 * 驾驶参考线信息（即实际执行的驾驶路径），障碍物预测等。所有决策都基于当前帧进行
 * 
 * 在Frame类中，主要的工作还是1、对障碍物预测轨迹(由Predition模块得到的未来5s内障碍物运动轨迹)、
 * 2、无人车参考线(ReferenceLineProvider类提供) 
 * 3、当前路况(停车标志、人行横道、减速带等)信息进行融合。
 */

class Frame {
 public:
 /*
 uint32_t sequence_num 帧信号，记录当前帧的信号
 */
  explicit Frame(uint32_t sequence_num,
                 const common::TrajectoryPoint &planning_start_point,
                 const double start_time,
                 const common::VehicleState &vehicle_state,
                 ReferenceLineProvider *reference_line_provider);

  const common::TrajectoryPoint &PlanningStartPoint() const;
  // 该函数被Planning::InitFrame函数调用，用于完成当前帧各类信息的初始化，
  // 包括获取高精地图、车辆状态，创建预测障碍、目的地障碍，查找碰撞障碍，初始化参考线信息等。
  common::Status Init();

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  const PublishableTrajectory &ComputedTrajectory() const;

  void RecordInputDebug(planning_internal::Debug *debug);

  std::list<ReferenceLineInfo> &reference_line_info();

  Obstacle *Find(const std::string &id);
// 该函数从可用的参考线信息列表中找到代价（cost）值最低的一条参考线信息，作为最终的驾驶路径。
  const ReferenceLineInfo *FindDriveReferenceLineInfo();

  const ReferenceLineInfo *DriveReferenceLineInfo() const;

  const std::vector<const Obstacle *> obstacles() const;

  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

  bool Rerouting();

  const common::VehicleState &vehicle_state() const;

  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);

  ADCTrajectory *mutable_trajectory() { return &trajectory_; }

  const ADCTrajectory &trajectory() const { return trajectory_; }

  const bool is_near_destination() const { return is_near_destination_; }

 private:
  bool CreateReferenceLineInfo();

  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such
   * obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;

  /**
   * @brief create a static virtual obstacle
   */
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);

  void AddObstacle(const Obstacle &obstacle);

 private:
//  帧序号 ---------- 记录当前帧序号
  uint32_t sequence_num_ = 0;
  // 高精地图类对象指针 ------------ 保存高精地图，给规划类对象提供导航地图及车辆定位点等信息。
  const hdmap::HDMap *hdmap_ = nullptr;
  // 轨迹起始点 ---------------- 保存路径的起始点
  common::TrajectoryPoint planning_start_point_;
  const double start_time_;
  common::VehicleState vehicle_state_;
//   参考线信息列表-------------存储多条候选路径
  std::list<ReferenceLineInfo> reference_line_info_;
  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
//  驾驶参考线信息指针 ------- 记录实际执行驾驶路径
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;
// 障碍物类对象 ------------- 存储车辆当前驾驶模式下，预测出现的障碍物
  prediction::PredictionObstacles prediction_;
  ThreadSafeIndexedObstacles obstacles_;
  ChangeLaneDecider change_lane_decider_;
  ADCTrajectory trajectory_;  // last published trajectory
  std::unique_ptr<LagPrediction> lag_predictor_;
  ReferenceLineProvider *reference_line_provider_ = nullptr;
  apollo::common::monitor::MonitorLogger monitor_logger_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_

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

#ifndef MODULES_PLANNING_PLANNING_H_
#define MODULES_PLANNING_PLANNING_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/traffic_rule_config.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/apollo_app.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planner/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 * 规划模块的主类，GPS和IMU作为输入处理，生成规划信息
 */
class Planning : public apollo::common::ApolloApp {
 public:
  Planning() = default;
  virtual ~Planning();
  /**
   * @brief module name
   * @return module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function  模块初始化函数
   * @return initialization status
   * 模块初始化功能。当应用程序启动时，这是第一个调用的函数
   * 通常这个函数加载配置，订阅来自传感器或其他模块的数据。
   */
  apollo::common::Status Init() override;

  /**
   * @brief module start function    模块开始函数
   * @return start status
   * 模块启动功能。阿波罗应用程序通常触发执行有两种方式：1.由上游消息触发或2.由定时器触发。
    如果应用程序是由上游消息触发的，Start（）函数通常会注册一个回调函数，当收到上游消息时将调用该函数。
    如果应用程序是由计时器触发的， Start（）函数通常会注册一个计时器回调函数。
   */
  apollo::common::Status Start() override;

  /**
   * @brief module stop function    模块停止函数
   */
  void Stop() override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   * 规划模块的主逻辑，定时触发运行。
   */
  void RunOnce();

  /**
   * @brief record last planning trajectory
   * 记录上次规划的轨迹
   */
  void SetLastPublishableTrajectory(const ADCTrajectory& adc_trajectory);

 private:
  // Watch dog timer 
  void OnTimer(const ros::TimerEvent&);

  void PublishPlanningPb(ADCTrajectory* trajectory_pb, double timestamp);

  /**
   * @brief Fill the header and publish the planning message.
   * 
   * 填充标题并发布计划消息
   */
  void Publish(planning::ADCTrajectory* trajectory) {
    using apollo::common::adapter::AdapterManager;
    AdapterManager::FillPlanningHeader(Name(), trajectory);
    AdapterManager::PublishPlanning(*trajectory);
  }

  void RegisterPlanners();

  /**
   * @brief Plan the trajectory given current vehicle state
   * 根据当前车辆状态规划轨迹
   */
  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* trajectory);

// 首先调用reset函数重新创建一个Frame类对象（若之前不存在，直接创建，若已经存在一个，先删除后创建），然后设置预测信息，
// 最后初始化帧
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const double start_time,
                           const common::VehicleState& vehicle_state);

  bool IsVehicleStateValid(const common::VehicleState& vehicle_state);
  void ExportReferenceLineDebug(planning_internal::Debug* debug);
  void SetFallbackTrajectory(ADCTrajectory* cruise_trajectory);

  /**
   * Reset pull over mode whenever received new routing
   * 每当接收到新路由时，重置pull-over模式
   */
  void ResetPullOver(const routing::RoutingResponse& response);

  void CheckPlanningConfig();

  double start_time_ = 0.0;
// 规划者工厂对象 -------EMPlanner,创建一个规划器
  common::util::Factory<PlanningConfig::PlannerType, Planner> planner_factory_;
// 规划配置类对象 -------- 读取配置文件信息，给规划类对象提供合适的参数
  PlanningConfig config_;

  TrafficRuleConfigs traffic_rule_configs_;
// 高精地图类对象指针 ---------- 该指针用于保存高精度地图，给规划类对象提供导航地图及车辆定位等信息；
  const hdmap::HDMap* hdmap_ = nullptr;
// 帧类对象指针 ---------- 用于保存帧Frame对象，规划类对象的所有决策都在该对象上完成；
  std::unique_ptr<Frame> frame_;
// 规划者类对象指针  ----------  用于保存具体的规划者对象；
  std::unique_ptr<Planner> planner_;
// 可发布轨迹类对象指针 ---------- 该指针用于记录规划路径的位置和速度信息
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;

  class VehicleConfig {
   public:
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    bool is_valid_ = false;
  };
  VehicleConfig last_vehicle_config_;
  VehicleConfig ComputeVehicleConfigFromLocalization(const localization::LocalizationEstimate& localization) const;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;

// 计时器对象 ----------  该对象用于定期监控各类操作用户，调用相关处理函数
  ros::Timer timer_;
  routing::RoutingResponse last_routing_;
};

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_PLANNING_H_ */

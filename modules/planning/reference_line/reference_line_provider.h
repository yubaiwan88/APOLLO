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
 * ReferenceLineProvider类是规划模块中参考线（即候选路径）的提供和管理者，
 * 它以Routing模块输出的高精度路径（从起点到终点的Lane片段集）为输入数据，
 * 经优化处理后生成平滑的参考线集给后续的动作规划子模块使用。
 * ReferenceLineProvider类是采用C++ 11标准实现的多线程类，
 * 每次调用ReferenceLineProvider::Start()函数后，该类就会在内部开启一个新线程，执行参考线的优化。
 * ReferenceLineProvider类是使用宏DECLARE_SINGLETON(ReferenceLineProvider)定义的单实例类，
 * 获取该类对象请使用ReferenceLineProvider::instance()。
 * @namespace apollo::planning
 * @file reference_line_provider.h
 *
 * @brief Declaration of the class ReferenceLineProvider.
 */

#ifndef MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
#define MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/util/factory.h"
#include "modules/common/util/util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/reference_line/spiral_reference_line_smoother.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */
class ReferenceLineProvider {
 public:
  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();

  explicit ReferenceLineProvider(const hdmap::HDMap* base_map);

  bool UpdateRoutingResponse(const routing::RoutingResponse& routing);

  void UpdateVehicleState(const common::VehicleState& vehicle_state);

  bool Start();

  void Stop();

  bool GetReferenceLines(std::list<ReferenceLine>* reference_lines,
                         std::list<hdmap::RouteSegments>* segments);

  double LastTimeDelay();

  std::vector<routing::LaneWaypoint> FutureRouteWaypoints();

  static double LookForwardDistance(const common::VehicleState& state);

 private:
  /**
   * @brief Use PncMap to create reference line and the corresponding segments
   * based on routing and current position. This is a thread safe function.
   * @return true if !reference_lines.empty() && reference_lines.size() ==
   *                 segments.size();
   **/
  bool CreateReferenceLine(std::list<ReferenceLine>* reference_lines,
                           std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief store the computed reference line. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */
  void UpdateReferenceLine(
      const std::list<ReferenceLine>& reference_lines,
      const std::list<hdmap::RouteSegments>& route_segments);

  void GenerateThread();
  void IsValidReferenceLine();
  void PrioritzeChangeLane(std::list<hdmap::RouteSegments>* route_segments);

  bool CreateRouteSegments(const common::VehicleState& vehicle_state,
                           const double look_forward_distance,
                           const double look_backward_distance,
                           std::list<hdmap::RouteSegments>* segments);

  bool IsReferenceLineSmoothValid(const ReferenceLine& raw,
                                  const ReferenceLine& smoothed) const;

  bool SmoothReferenceLine(const ReferenceLine& raw_reference_line,
                           ReferenceLine* reference_line);

  bool SmoothPrefixedReferenceLine(const ReferenceLine& prefix_ref,
                                   const ReferenceLine& raw_ref,
                                   ReferenceLine* reference_line);

  void GetAnchorPoints(const ReferenceLine& reference_line,
                       std::vector<AnchorPoint>* anchor_points) const;

  bool SmoothRouteSegment(const hdmap::RouteSegments& segments,
                          ReferenceLine* reference_line);

  /**
   * @brief This function creates a smoothed forward reference line
   * based on the given segments.
   */
  bool ExtendReferenceLine(const common::VehicleState& state,
                           hdmap::RouteSegments* segments,
                           ReferenceLine* reference_line);

  AnchorPoint GetAnchorPoint(const ReferenceLine& reference_line,
                             double s) const;

  bool GetReferenceLinesFromRelativeMap(
      const relative_map::MapMsg& relative_map,
      std::list<ReferenceLine>* reference_line,
      std::list<hdmap::RouteSegments>* segments);

  /**
   * @brief This function get adc lane info from navigation path and map
   * by vehicle state.
   */
  bool GetNearestWayPointFromNavigationPath(
      const common::VehicleState& state,
      const std::unordered_set<std::string>& navigation_lane_ids,
      hdmap::LaneWaypoint* waypoint);

 private:
  bool is_initialized_ = false;
  bool is_stop_ = false;
  std::unique_ptr<std::thread> thread_;

  std::unique_ptr<ReferenceLineSmoother> smoother_;
//   二次规划样条参考线平滑配置类对象指针
  QpSplineReferenceLineSmootherConfig smoother_config_;

  std::mutex pnc_map_mutex_;
//   规划控制地图类对象智能指针 ---------------- 规划控制模块使用的地图数据类对象
  std::unique_ptr<hdmap::PncMap> pnc_map_;

  std::mutex vehicle_state_mutex_;
  common::VehicleState vehicle_state_;

  std::mutex routing_mutex_;
//   路由寻径响应类对象 ------------- Routing模块输出的路由寻径数据
  routing::RoutingResponse routing_;
//   是否具备路由 ---------- 从起点到终点是否具备路由
  bool has_routing_ = false;
// 参考线互斥体对象 -------------- 允许参考线在多线程环境下安全使用的互斥体类对象。
  std::mutex reference_lines_mutex_;
//   参考线列表类对象 ---------- 存储多条参考线
  std::list<ReferenceLine> reference_lines_;
//   路由片段列表类对象 ---------- 存储多条路由片段
  std::list<hdmap::RouteSegments> route_segments_;
//   上次计算时间 ------------- 上次计算平滑参考线耗时
  double last_calculation_time_ = 0.0;

  std::queue<std::list<ReferenceLine>> reference_line_history_;
  std::queue<std::list<hdmap::RouteSegments>> route_segments_history_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
